/*
 * UBX driver.  For u-blox binary, also includes Antaris4 binary
 * Reference manuals are at
 * http://www.u-blox.com/en/download/documents-a-resources/u-blox-6-gps-modules-resources.html
 *
 * updated for u-blox 8
 * http://www.ublox.com/images/downloads/Product_Docs/u-bloxM8_ReceiverDescriptionProtocolSpec_%28UBX-13003221%29_Public.pdf
 *
 * Week counters are not limited to 10 bits. It's unknown what
 * the firmware is doing to disambiguate them, if anything; it might just
 * be adding a fixed offset based on a hidden epoch value, in which case
 * unhappy things will occur on the next rollover.
 *
 * For the Antaris 4, the default leap-second offset (before getting one from
 * the sats, one presumes) is 0sec; for the u-blox 6 it's 15sec.
 *
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 *
 */

#include "gpsd_config.h" /* must be before all includes */

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "driver_ubx.h"
#include "gpsd.h"
#include "gps.h"

#include "bits.h" // For UINT2INT()
#include "timespec.h"

#include "../gps-sim.h"

#include "../others/log.h"

/*
 * Some high-precision messages provide data where the main part is a
 * signed 32-bit integer (same as the standard-precision versions),
 * and there's an 8-bit signed field providing an addend scaled to
 * 1/100th of the main value.  This macro provides a fetch for such
 * values, scaled to match the extension (i.e., 100X the main-value scale).
 * Since the fields are nonconsective, the offsets are provided separately.
 * The result is a signed 64-bit integer.
 *
 * The second macro incorporates scaling the result by a specified double.
 */
#define getles32x100s8(buf, off, offx) \
    ((int64_t)(getles32((buf), (off)) * 100LL + getsb((buf), (offx))))
#define getles32x100s8d(buf, off, offx, scale) \
    (getles32x100s8((buf), (off), (offx)) * (double)(scale))

/*
 * A ubx packet looks like this:
 * leader: 0xb5 0x62
 * message class: 1 byte
 * message type: 1 byte
 * length of payload: 2 bytes
 * payload: variable length
 * checksum: 2 bytes
 *
 * see also the FV25 and UBX documents on reference.html
 */
#define UBX_PREFIX_LEN 6
#define UBX_CLASS_OFFSET 2
#define UBX_TYPE_OFFSET 3

/* because we hates magic numbers forever */
#define USART1_ID 1
#define USART2_ID 2
#define USB_ID 3
#define UBX_PROTOCOL_MASK 0x01
#define NMEA_PROTOCOL_MASK 0x02
#define RTCM_PROTOCOL_MASK 0x04
#define RTCM3_PROTOCOL_MASK 0x20 // protVer 20+
#define UBX_CFG_LEN 20
#define outProtoMask 14

typedef struct
{
    const char *fw_string;
    const float protver;
} fw_protver_map_entry_t;

/* based on u-blox document no. GPS.G7-SW-12001-B1 (15 June 2018) */
/* capture decimal parts of protVer info even when session->protver currently
 * is integer (which _might_ change in the future, so avoid having to revisit
 * the info at that time).
 * This list is substantially incomplete and over specific. */
const fw_protver_map_entry_t fw_protver_map[] = {
    {"2.10", 8.10},  // antaris 4, version 8 is a guess
    {"2.11", 8.11},  // antaris 4, version 8 is a guess
    {"3.04", 9.00},  // antaris 4, version 9 is a guess
    {"4.00", 10.00}, // antaris 4, and u-blox 5
    {"4.01", 10.01}, // antaris 4, and u-blox 5
    {"5.00", 11.00}, // u-blox 5 and antaris 4
    {"6.00", 12.00}, // u-blox 5 and 6
    {"6.02", 12.02}, // u-blox 5 and 6
    {"7.01", 13.01}, // u-blox 7
    {"7.03", 13.03}, // u-blox 7
    {"1.00", 14.00}, // u-blox 6 w/ GLONASS, and 7
    // protVer >14 should carry explicit protVer in MON-VER extension
    {NULL, 0.0},
};

/*
 * Model  Fw          Protver
 * M10    SPG 5.00    34.00
 */

/* Convert a ubx PRN to an NMEA 4.0 (extended) PRN and ubx gnssid, svid
 *
 * return 0 on fail
 */
short ubx_to_prn(int ubx_PRN, unsigned char *gnssId,
                 unsigned char *svId)
{
    *gnssId = 0;
    *svId = 0;

    // IRNSS??
    if (1 > ubx_PRN)
    {
        // skip 0 PRN
        return 0;
    }
    else if (32 >= ubx_PRN)
    {
        // GPS 1..32 -> 1..32
        *gnssId = 0;
        *svId = ubx_PRN;
    }
    else if (64 >= ubx_PRN)
    {
        // BeiDou, 159..163,33..64 -> 1..5,6..37
        *gnssId = 3;
        *svId = ubx_PRN - 27;
    }
    else if (96 >= ubx_PRN)
    {
        // GLONASS 65..96 -> 1..32
        *gnssId = 6;
        *svId = ubx_PRN - 64;
    }
    else if (120 > ubx_PRN)
    {
        // Huh?
        return 0;
    }
    else if (158 >= ubx_PRN)
    {
        // SBAS 120..158 -> 120..158
        *gnssId = 1;
        *svId = ubx_PRN;
    }
    else if (163 >= ubx_PRN)
    {
        // BeiDou, 159..163 -> 1..5
        *gnssId = 3;
        *svId = ubx_PRN - 158;
    }
    else if (173 > ubx_PRN)
    {
        // Huh?
        return 0;
    }
    else if (182 >= ubx_PRN)
    {
        // IMES 173..182 -> 1..5, in u-blox 8, bot u-blox 9
        *gnssId = 4;
        *svId = ubx_PRN - 172;
    }
    else if (193 > ubx_PRN)
    {
        // Huh?
        return 0;
    }
    else if (199 >= ubx_PRN)
    {
        // QZSS 193..197 -> 1..5
        // ZED-F9T also see 198 and 199
        *gnssId = 5;
        *svId = ubx_PRN - 192;
    }
    else if (211 > ubx_PRN)
    {
        // Huh?
        return 0;
    }
    else if (246 >= ubx_PRN)
    {
        // Galileo 211..246 -> 1..36
        *gnssId = 2;
        *svId = ubx_PRN - 210;
    }
    else
    {
        // greater than 246, GLONASS (255), unused, or other unknown
        return 0;
    }
    return ubx2_to_prn(*gnssId, *svId);
}

// UBX-CFG-RATE
// Deprecated in u-blox 10
void ubx_msg_cfg_rate(struct gps_device_t *session, unsigned char *buf,
                      size_t data_len)
{
    return;
}

/* UBX-ESF-ALG
 *
 * UBX-ESF-ALG, and UBX-ESF-INS are synchronous to the GNSS epoch.
 * They need to be combined and reported together with the rest of
 * the epoch.
 */
gps_mask_t
ubx_msg_esf_alg(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{

    return 0;
}

/* UBX-ESF-INS
 *
 * protVer 19 and up.  ADR and UDR only
 *
 * UBX-ESF-ALG, and UBX-ESF-INS are synchronous to the GNSS epoch.
 * They need to be combined and reported together with the rest of
 * the epoch.
 */
gps_mask_t
ubx_msg_esf_ins(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    return 0;
}

/* UBX-ESF-MEAS
 *
 * protVer 15 and up.  ADR only
 * protVer 19 and up.  ADR and UDR only
 *
 * asynchronous to the GNSS epoch, and at a higher rate.
 * Needs to be reported immediately.
 *
 */
gps_mask_t
ubx_msg_esf_meas(struct gps_device_t *session, unsigned char *buf,
                 size_t data_len)
{
    return 0;
}

/* UBX-ESF-RAW
 *
 * protVer 15 and up.  ADR only
 * protVer 19 and up.  ADR and UDR only
 *
 * asynchronous to the GNSS epoch, and a a higher rate.
 * Needs to be reported immediately.
 *
 */
gps_mask_t
ubx_msg_esf_raw(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    return 0;
}

// UBX-ESF-STATUS
gps_mask_t
ubx_msg_esf_status(struct gps_device_t *session, unsigned char *buf,
                   size_t data_len)
{
    return 0;
}

/**
 * HNR Attitude solution
 * UBX-HNR-ATT Class x28, ID 1
 *
 * Not before u-blox 8, protVer 19.2 and up.
 * only on ADR, and UDR
 */
gps_mask_t
ubx_msg_hnr_att(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    return 0;
}

/**
 * HNR Vehicle dynamics information
 * UBX-HNR-INS Class x28, ID 2
 *
 * Not before u-blox 8, protVer 19.1 and up.
 * only on ADR, and UDR
 */
gps_mask_t
ubx_msg_hnr_ins(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    return 0;
}

/**
 * High rate output of PVT solution
 * UBX-HNR-PVT Class x28, ID 2
 *
 * Not before u-blox 8, protVer 19 and up.
 * only on ADR, and UDR
 */
gps_mask_t
ubx_msg_hnr_pvt(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    return 0;
}

/**
 * Receiver/Software Version
 * UBX-MON-VER
 *
 * sadly more info than fits in session->swtype for now.
 * so squish the data hard.
 */
gps_mask_t ubx_msg_mon_ver(struct gps_device_t *session,
                           unsigned char *buf,
                           size_t data_len)
{
    // output SW and HW Version at LOG_INF
    return 0;
}

/* UBX-MON-TXBUF
 * Present in u-blox 5+ through at least protVer 23.01
 * Supported but deprecated in M9P protVer 27.11
 * Supported but deprecated in M9N protVer 32.00 */
gps_mask_t
ubx_msg_mon_txbuf(struct gps_device_t *session, unsigned char *buf,
                  size_t data_len)
{
    return 0;
}

/* UBX-MON-RXBUF
 * Present in u-blox 5+ through at least protVer 23.01
 * Supported but deprecated in M9P protVer 27.11
 * Supported but deprecated in M9N protVer 32.00 */
gps_mask_t
ubx_msg_mon_rxbuf(struct gps_device_t *session, unsigned char *buf,
                  size_t data_len)
{
    return 0;
}

/**
 * UBX-LOG-BATCH entry only part of UBX protocol
 * Used for GPS standalone operation (internal batch retrieval)
 */
gps_mask_t
ubx_msg_log_batch(struct gps_device_t *session, unsigned char *buf,
                  size_t data_len)
{
    return 0;
}

/**
 * UBX-LOG-INFO info of log status
 * u-blox 7,8,9.  protVer 14 to 29
 * WIP: Initial decode, log only.
 *
 */
gps_mask_t
ubx_msg_log_info(struct gps_device_t *session, unsigned char *buf,
                 size_t data_len)
{
    return 0;
}

/*
 * UBX-LOG-RETRIEVEPOS (Indexed PVT entry)
 * Used for GPS standalone operation and host saved logs
 * u-blox 7,8,9.  protVer 14 to 29
 */
gps_mask_t
ubx_msg_log_retrievepos(struct gps_device_t *session, unsigned char *buf,
                        size_t data_len)
{
    return 0;
}

/*
 * UBX-LOG-RETRIEVEPOSEXTRA (Indexed Odometry entry)
 * Used for GPS standalone operation and host saved logs
 * u-blox 7,8,9.  protVer 14 to 29
 */
gps_mask_t
ubx_msg_log_retrieveposextra(struct gps_device_t *session,
                             unsigned char *buf, size_t data_len)
{
    return 0;
}

/*
 * UBX-NAV-HPPOSECEF - High Precision Position Solution in ECEF
 *
 * Present in u-blox 8 and above, protVwer 20.00 and up.
 * Only with High Precision firmware.
 */
gps_mask_t
ubx_msg_nav_hpposecef(struct gps_device_t *session, unsigned char *buf,
                      size_t data_len)
{
    return 0;
}

/**
 * High Precision Geodetic Position Solution
 * UBX-NAV-HPPOSLLH, Class 1, ID x14
 *
 * No mode, so limited usefulness.
 *
 * Present in u-blox 8 and above, protVwer 20.00 and up.
 * Only with High Precision firmware.
 */
gps_mask_t
ubx_msg_nav_hpposllh(struct gps_device_t *session, unsigned char *buf,
                     size_t data_len)
{
    return 0;
}

/*
 * Navigation Position ECEF message
 *
 * This message does not bother to tell us if it is valid.
 */
gps_mask_t
ubx_msg_nav_posecef(struct gps_device_t *session, unsigned char *buf,
                    size_t data_len)
{
    return 0;
}

/**
 * Navigation Position Velocity Time solution message
 * UBX-NAV-PVT Class 1, ID 7
 *
 * Not in u-blox 5 or 6, present in u-blox 7
 * u-blox 6 w/ GLONASS, protver 14 have NAV-PVT
 */
gps_mask_t
ubx_msg_nav_pvt(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    uint8_t valid;
    uint8_t flags;
    uint8_t fixType;
    struct tm unpacked_date;
    int *status = &session->gpsdata.fix.status;
    int *mode = &session->gpsdata.fix.mode;
    gps_mask_t mask = 0;
    char ts_buf[TIMESPEC_LEN];

    /* u-blox 6 and 7 are 84 bytes, u-blox 8 and 9 are 92 bytes  */
    if (84 > data_len)
    {
        return 0;
    }

    session->iTOW = getleu32(buf, 0);
    valid = (unsigned int)getub(buf, 11);
    fixType = (unsigned int)getub(buf, 20);
    flags = (unsigned int)getub(buf, 21);

    session->gpsdata.fix.mode = fixType;

    if ((valid & UBX_NAV_PVT_VALID_DATE_TIME) == UBX_NAV_PVT_VALID_DATE_TIME)
    {
        unpacked_date.tm_year = (uint16_t)getleu16(buf, 4) - 1900;
        unpacked_date.tm_mon = (uint8_t)getub(buf, 6) - 1;
        unpacked_date.tm_mday = (uint8_t)getub(buf, 7);
        unpacked_date.tm_hour = (uint8_t)getub(buf, 8);
        unpacked_date.tm_min = (uint8_t)getub(buf, 9);
        unpacked_date.tm_sec = (uint8_t)getub(buf, 10);
        unpacked_date.tm_isdst = 0;
        unpacked_date.tm_wday = 0;
        unpacked_date.tm_yday = 0;
        session->gpsdata.fix.time.tv_sec = mkgmtime(&unpacked_date);
        /* field 16, nano, can be negative! So normalize */
        session->gpsdata.fix.time.tv_nsec = getles32(buf, 16);
        TS_NORM(&session->gpsdata.fix.time);
        mask |= TIME_SET | NTPTIME_IS | GOODTIME_IS;
    }

    session->gpsdata.fix.longitude = 1e-7 * getles32(buf, 24);
    session->gpsdata.fix.latitude = 1e-7 * getles32(buf, 28);
    /* altitude WGS84 */
    session->gpsdata.fix.altHAE = 1e-3 * getles32(buf, 32);
    /* altitude MSL */
    session->gpsdata.fix.altMSL = 1e-3 * getles32(buf, 36);
    /* Let gpsd_error_model() deal with geoid_sep */

    session->gpsdata.fix.speed = 1e-3 * (int32_t)getles32(buf, 60);
    /* u-blox calls this Heading of motion (2-D) */
    session->gpsdata.fix.track = 1e-5 * (int32_t)getles32(buf, 64);
    mask |= LATLON_SET | ALTITUDE_SET | SPEED_SET | TRACK_SET;

    /* u-blox does not document the basis for the following "accuracy"
     * estimates.  Maybe CEP(50), one sigma, two sigma, CEP(99), etc. */

    /* Horizontal Accuracy estimate, in mm */
    session->gpsdata.fix.eph = (double)(getles32(buf, 40) / 1000.0);
    /* Vertical Accuracy estimate, in mm */
    session->gpsdata.fix.epv = (double)(getles32(buf, 44) / 1000.0);
    /* Speed Accuracy estimate, in mm/s */
    session->gpsdata.fix.eps = (double)(getles32(buf, 68) / 1000.0);
    /* let gpsd_error_model() do the rest */

    mask |= HERR_SET | SPEEDERR_SET | VERR_SET;
    // if cycle ender worked, could get rid of this REPORT_SET.
    // mask |= REPORT_SET;
    if (92 <= data_len)
    {
        // u-blox 8 and 9 extended
        double magDec = NAN;
        double magAcc = NAN;
#ifdef __UNUSED
        if (flags & UBX_NAV_PVT_FLAG_HDG_OK)
        {
            /* u-blox calls this Heading of vehicle (2-D)
             * why is it different than earlier track? */
            session->gpsdata.fix.track = (double)(getles32(buf, 84) * 1e-5);
        }
#endif // __UNUSED
        if (valid & UBX_NAV_PVT_VALID_MAG)
        {
            magDec = (double)(getles16(buf, 88) * 1e-2);
            magAcc = (double)(getleu16(buf, 90) * 1e-2);
        }
    }
    return mask;
}

/**
 * High Precision Relative Positioning Information in NED frame
 * UBX-NAV-RELPOSNED, Class 1, ID x3c
 * HP GNSS only, protver 20+
 */
gps_mask_t
ubx_msg_nav_relposned(struct gps_device_t *session, unsigned char *buf,
                      size_t data_len)
{
    return 0;
}

/**
 * Navigation solution message: UBX-NAV-SOL
 *
 * UBX-NAV-SOL, present in Antaris, up to 23,01
 * deprecated in u-blox 6, gone in u-blox 9.
 * Use UBX-NAV-PVT instead
 *
 * UBX-NAV-SOL has ECEF and VECEF, so no need for UBX-NAV-POSECEF and
 * UBX-NAV-VELECEF
 */
gps_mask_t ubx_msg_nav_sol(struct gps_device_t *session,
                           unsigned char *buf, size_t data_len)
{
    unsigned flags, pdop;
    unsigned char navmode;
    gps_mask_t mask;
    char ts_buf[TIMESPEC_LEN];

    if (52 > data_len)
    {
        return 0;
    }

    session->iTOW = getleu32(buf, 0);
    flags = (unsigned int)getub(buf, 11);
    mask = 0;
#define DATE_VALID (UBX_SOL_VALID_WEEK | UBX_SOL_VALID_TIME)
    if ((flags & DATE_VALID) == DATE_VALID)
    {
        unsigned short week;
        timespec_t ts_tow;

        MSTOTS(&ts_tow, session->iTOW);
        ts_tow.tv_nsec += (long)getles32(buf, 4);
        TS_NORM(&ts_tow);
        week = (unsigned short)getles16(buf, 8);
        // session->gpsdata.fix.time = gpsd_gpstime_resolv(session, week, ts_tow);
        mask |= TIME_SET | NTPTIME_IS | GOODTIME_IS;
    }
#undef DATE_VALID

    session->gpsdata.fix.ecef.x = getles32(buf, 12) / 100.0;
    session->gpsdata.fix.ecef.y = getles32(buf, 16) / 100.0;
    session->gpsdata.fix.ecef.z = getles32(buf, 20) / 100.0;
    session->gpsdata.fix.ecef.pAcc = getleu32(buf, 24) / 100.0;
    session->gpsdata.fix.ecef.vx = getles32(buf, 28) / 100.0;
    session->gpsdata.fix.ecef.vy = getles32(buf, 32) / 100.0;
    session->gpsdata.fix.ecef.vz = getles32(buf, 36) / 100.0;
    session->gpsdata.fix.ecef.vAcc = getleu32(buf, 40) / 100.0;
    mask |= ECEF_SET | VECEF_SET;

    session->gpsdata.fix.eps = (double)(getles32(buf, 40) / 100.0);
    mask |= SPEEDERR_SET;

    pdop = getleu16(buf, 44);
    if (9999 > pdop)
    {
        session->gpsdata.dop.pdop = (double)(pdop / 100.0);
        mask |= DOP_SET;
    }
    session->gpsdata.satellites_used = (int)getub(buf, 47);

    navmode = (unsigned char)getub(buf, 10);
    switch (navmode)
    {
    case UBX_MODE_TMONLY:
        // Surveyed-in, better not have moved
        session->gpsdata.fix.mode = MODE_3D;
        session->gpsdata.fix.status = STATUS_TIME;
        break;
    case UBX_MODE_3D:
        session->gpsdata.fix.mode = MODE_3D;
        session->gpsdata.fix.status = STATUS_GPS;
        break;
    case UBX_MODE_2D:
        session->gpsdata.fix.mode = MODE_2D;
        session->gpsdata.fix.status = STATUS_GPS;
        break;
    case UBX_MODE_DR: // consider this too as 2D
        session->gpsdata.fix.mode = MODE_2D;
        session->gpsdata.fix.status = STATUS_DR;
        break;
    case UBX_MODE_GPSDR: // DR-aided GPS is valid 3D
        session->gpsdata.fix.mode = MODE_3D;
        session->gpsdata.fix.status = STATUS_GNSSDR;
        break;
    default:
        session->gpsdata.fix.mode = MODE_NO_FIX;
        session->gpsdata.fix.status = STATUS_UNK;
        break;
    }

    if (0 != (flags & UBX_SOL_FLAG_DGPS))
        session->gpsdata.fix.status = STATUS_DGPS;

    mask |= MODE_SET | STATUS_SET;
    // older u-blox, cycle ender may be iffy
    // so err o nthe side of over-reporting TPV
    mask |= REPORT_SET;
    return mask;
}

/**
 * Receiver navigation status
 * UBX-NAV-STATUS Class 1, ID 3
 *
 * Present in Antaris to 9-series
 */
gps_mask_t
ubx_msg_nav_status(struct gps_device_t *session, unsigned char *buf,
                   size_t data_len)
{
    return 0;
}

/**
 * Navigation time to leap second: UBX-NAV-TIMELS
 *
 * Sets leap_notify if leap second is < 23 hours away.
 * Not in u-blox 5
 */
gps_mask_t
ubx_msg_nav_timels(struct gps_device_t *session, unsigned char *buf,
                   size_t data_len)
{
    return 0;
}

/**
 * Geodetic position solution message
 * UBX-NAV-POSLLH, Class 1, ID 2
 *
 * This message does not bother to tell us if it is valid.
 * No mode, so limited usefulness
 */
gps_mask_t
ubx_msg_nav_posllh(struct gps_device_t *session, unsigned char *buf,
                   size_t data_len)
{
    gps_mask_t mask = 0;

    if (28 > data_len)
    {
        return 0;
    }

    session->iTOW = getleu32(buf, 0);
    session->gpsdata.fix.longitude = 1e-7 * getles32(buf, 4);
    session->gpsdata.fix.latitude = 1e-7 * getles32(buf, 8);
    /* altitude WGS84 */
    session->gpsdata.fix.altHAE = 1e-3 * getles32(buf, 12);
    /* altitude MSL */
    session->gpsdata.fix.altMSL = 1e-3 * getles32(buf, 16);
    /* Let gpsd_error_model() deal with geoid_sep */

    /* Horizontal accuracy estimate in mm, unknown type */
    session->gpsdata.fix.eph = getleu32(buf, 20) * 1e-3;
    /* Vertical accuracy estimate in mm, unknown type */
    session->gpsdata.fix.epv = getleu32(buf, 24) * 1e-3;

    mask = ONLINE_SET | HERR_SET | VERR_SET | LATLON_SET | ALTITUDE_SET;
    return mask;
}

/**
 * Clock Solution
 *
 * Present in u-blox 7
 */
gps_mask_t
ubx_msg_nav_clock(struct gps_device_t *session, unsigned char *buf,
                  size_t data_len)
{
    gps_mask_t mask = CLOCK_SET;

    if (20 > data_len)
    {
        return 0;
    }

    session->iTOW = getleu32(buf, 0);
    session->gpsdata.fix.clock_bias = getles32(buf, 4);
    session->gpsdata.fix.clock_drift = getles32(buf, 8);
    session->gpsdata.fix.tAcc_estimate = getleu32(buf, 12);
    session->gpsdata.fix.fAcc_estimate = getleu32(buf, 16);
    return mask;
}

/**
 * DGPS Data Used for NAV
 *
 * May be good cycle ender
 *
 * Present in u-blox 7
 */
gps_mask_t ubx_msg_nav_dgps(struct gps_device_t *session,
                            unsigned char *buf, size_t data_len)
{
    return 0;
}

/**
 * Dilution of precision message
 */
gps_mask_t
ubx_msg_nav_dop(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    unsigned u;
    gps_mask_t mask = 0;

    if (18 > data_len)
    {
        return 0;
    }

    session->iTOW = getleu32(buf, 0);
    /*
     * We make a deliberate choice not to clear DOPs from the
     * last skyview here, but rather to treat this as a supplement
     * to our calculations from the visibility matrix, trusting
     * the firmware algorithms over ours.
     */
    u = getleu16(buf, 4);
    if (9999 > u)
    {
        session->gpsdata.dop.gdop = (double)(u / 100.0);
        mask |= DOP_SET;
    }
    u = getleu16(buf, 6);
    if (9999 > u)
    {
        session->gpsdata.dop.pdop = (double)(u / 100.0);
        mask |= DOP_SET;
    }
    u = getleu16(buf, 8);
    if (9999 > u)
    {
        session->gpsdata.dop.tdop = (double)(u / 100.0);
        mask |= DOP_SET;
    }
    u = getleu16(buf, 10);
    if (9999 > u)
    {
        session->gpsdata.dop.vdop = (double)(u / 100.0);
        mask |= DOP_SET;
    }
    u = getleu16(buf, 12);
    if (9999 > u)
    {
        session->gpsdata.dop.hdop = (double)(u / 100.0);
        mask |= DOP_SET;
    }
    return mask;
}

/**
 * Position error ellipse parameters
 * protVer 19.1 and up
 * Not in u-blox 5, 6 or 7
 * Present in some u-blox 8, 9 and 10 (ADR, HPS)
 */
gps_mask_t
ubx_msg_nav_eell(struct gps_device_t *session, unsigned char *buf,
                 size_t data_len)
{
    return 0;
}

/**
 * End of Epoch
 * Not in u-blox 5, 6 or 7
 * Present in u-blox 8 and 9
 */
gps_mask_t
ubx_msg_nav_eoe(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    gps_mask_t mask = 0;

    if (4 > data_len)
    {
        return 0;
    }
    /* nothing to report, but the iTOW for cycle ender is good */
    mask |= REPORT_SET;
    return mask;
}

/**
 * GPS Leap Seconds - UBX-NAV-TIMEGPS
 */
gps_mask_t
ubx_msg_nav_timegps(struct gps_device_t *session, unsigned char *buf,
                    size_t data_len)
{
    uint8_t valid; /* Validity Flags */
    gps_mask_t mask = 0;
    char ts_buf[TIMESPEC_LEN];

    if (16 > data_len)
    {
        return 0;
    }

    session->iTOW = getleu32(buf, 0);
    valid = getub(buf, 11);
    // Valid leap seconds ?
    if ((valid & UBX_TIMEGPS_VALID_LEAP_SECOND) ==
        UBX_TIMEGPS_VALID_LEAP_SECOND)
    {
        // session->context->leap_seconds = (int)getub(buf, 10);
        // session->context->valid |= LEAP_SECOND_VALID;
    }
    // Valid GPS time of week and week number
#define VALID_TIME (UBX_TIMEGPS_VALID_TIME | UBX_TIMEGPS_VALID_WEEK)
    if ((valid & VALID_TIME) == VALID_TIME)
    {
#undef VALID_TIME
        uint16_t week;
        double tAcc; /* Time Accuracy Estimate in ns */
        timespec_t ts_tow;

        week = getles16(buf, 8);
        MSTOTS(&ts_tow, session->iTOW);
        ts_tow.tv_nsec += (long)getles32(buf, 4);
        TS_NORM(&ts_tow);
        session->gpsdata.fix.gps_time_itow = ts_tow.tv_sec;
        session->gpsdata.fix.gps_time_ftow = ts_tow.tv_nsec;
        session->gpsdata.fix.gps_time_weekn = week;
        // session->gpsdata.fix.time = gpsd_gpstime_resolv(session, week, ts_tow);

        tAcc = (double)getleu32(buf, 12); // tAcc in ns
        session->gpsdata.fix.ept = tAcc * 1e-9;

        mask |= (GPSTIME_SET | TIME_SET | NTPTIME_IS);
    }

    return mask;
}

/**
 * UBX-NAV-TIMEGAL
 */

gps_mask_t
ubx_msg_nav_timegal(struct gps_device_t *session, unsigned char *buf,
                    size_t data_len)
{
    return 0;
}

/**
 * UBX-NAV-TIMEUTC
 */
gps_mask_t
ubx_msg_nav_timeutc(struct gps_device_t *session, unsigned char *buf,
                    size_t data_len)
{
    uint8_t valid; // Validity Flags
    gps_mask_t mask = 0;

    if (20 > data_len)
    {
        return 0;
    }

    session->iTOW = getleu32(buf, 0);
    valid = getub(buf, 19);
    if (4 == (4 & valid))
    {
        // UTC is valid
        struct tm date = {0};
        // mask |= (TIME_SET | NTPTIME_IS);
        uint32_t tAcc = getleu32(buf, 4); // tAcc in ns
        // nano can be negative, so this is not normalized UTC.
        int32_t nano = getles32(buf, 8);         // fract sec in ns
        date.tm_year = getleu16(buf, 12) - 1900; // year, 1999..2099
        date.tm_mon = getub(buf, 14) - 1;        // month 1..12
        date.tm_mday = getub(buf, 15);           // day 1..31
        date.tm_hour = getub(buf, 16);           // hour 0..23
        date.tm_min = getub(buf, 17);            // min 0..59
        date.tm_sec = getub(buf, 18);            // sec 0..60
        session->gpsdata.fix.time.tv_sec = mkgmtime(&date);
        session->gpsdata.fix.time.tv_nsec = nano;
        // nano, can be negative! So normalize
        TS_NORM(&session->gpsdata.fix.time);
        // other timestamped messages lack nano, so time will jump around...
        mask |= TIME_SET | NTPTIME_IS | GOODTIME_IS;
    }
    return mask;
}

/**
 * GPS Satellite Info -- new style UBX-NAV-SAT
 * Not in u-blox 5
 * Present in u-blox 8,  protocol version 15+
 */
gps_mask_t
ubx_msg_nav_sat(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    unsigned int i, nchan, nsv, ver;
    timespec_t ts_tow;

    if (8 > data_len)
    {
        return 0;
    }

    session->iTOW = getleu32(buf, 0);
    MSTOTS(&ts_tow, session->iTOW);

    ver = (unsigned int)getub(buf, 4);
    if (1 != ver)
    {
        return 0;
    }
    nchan = (unsigned int)getub(buf, 5);
    if (nchan > MAXCHANNELS)
    {
        return 0;
    }
    /* two "unused" bytes at buf[6:7] */
    nsv = 0;
    for (i = 0; i < nchan; i++)
    {
        unsigned int off = 8 + 12 * i;
        short nmea_PRN = 0;
        unsigned char gnssId = getub(buf, off + 0);
        short svId = (short)getub(buf, off + 1);
        unsigned char cno = getub(buf, off + 2);
        /* health data in flags. */
        uint32_t flags = getleu32(buf, off + 16);
        bool used = (bool)(flags & 0x08);
        int tmp;
        /* Notice NO sigid! */

        nmea_PRN = ubx2_to_prn(gnssId, svId);
        session->gpsdata.skyview[svId].gnssid = gnssId;
        session->gpsdata.skyview[svId].svid = svId;
        session->gpsdata.skyview[svId].PRN = nmea_PRN;

        session->gpsdata.skyview[svId].ss = (double)cno;
        tmp = getsb(buf, off + 3);
        if (90 >= abs(tmp))
        {
            session->gpsdata.skyview[svId].elevation = (double)tmp;
        }
        tmp = getles16(buf, off + 4);
        if (359 >= tmp && 0 <= tmp)
        {
            session->gpsdata.skyview[svId].azimuth = (double)tmp;
        }
        session->gpsdata.skyview[svId].used = used;
        /* by some coincidence, our health flags matches u-blox's */
        session->gpsdata.skyview[svId].health = (flags >> 4) & 3;
        session->gpsdata.skyview[svId].qi = (flags)&3;
        /* sbas_in_use is not same as used */
        if (used)
        {
            nsv++;
        }
    }

    session->gpsdata.satellites_visible = (int)nchan;
    session->gpsdata.satellites_used = (int)nsv;
    return SATELLITE_SET | USED_IS;
}

/**
 * GPS Satellite Info -- deprecated - UBX-NAV-SVINFO
 * Not in u-blox 9 or 10, use UBX-NAV-SAT instead
 */
gps_mask_t
ubx_msg_nav_svinfo(struct gps_device_t *session, unsigned char *buf,
                   size_t data_len)
{
    unsigned int i, nchan, nsv, st;
    timespec_t ts_tow;

    if (8 > data_len)
    {

        return 0;
    }

    session->iTOW = getleu32(buf, 0);
    MSTOTS(&ts_tow, session->iTOW);

    nchan = (unsigned int)getub(buf, 4);
    if (nchan > MAXCHANNELS)
    {
        return 0;
    }
    nsv = 0;
    for (i = st = 0; i < nchan; i++)
    {
        unsigned int off = 8 + 12 * i;
        short nmea_PRN;
        short ubx_PRN = (short)getub(buf, off + 1);
        unsigned char snr = getub(buf, off + 4);
        bool used = (bool)(getub(buf, off + 2) & 0x01);
        unsigned char flags = getub(buf, off + 12) & 3;
        int tmp;

        nmea_PRN = ubx_to_prn(ubx_PRN,
                              &session->gpsdata.skyview[st].gnssid,
                              &session->gpsdata.skyview[st].svid);

        if (1 > nmea_PRN)
        {
            // skip bad PRN
            continue;
        }
        session->gpsdata.skyview[st].PRN = nmea_PRN;

        session->gpsdata.skyview[st].ss = (double)snr;
        tmp = getsb(buf, off + 5);
        if (90 >= abs(tmp))
        {
            session->gpsdata.skyview[st].elevation = (double)tmp;
        }
        tmp = (double)getles16(buf, off + 6);
        if (359 > tmp && 0 <= tmp)
        {
            session->gpsdata.skyview[st].azimuth = (double)tmp;
        }
        session->gpsdata.skyview[st].used = used;
        if (0x10 & flags)
        {
            session->gpsdata.skyview[st].health = SAT_HEALTH_BAD;
        }
        else
        {
            session->gpsdata.skyview[st].health = SAT_HEALTH_OK;
        }

        /* sbas_in_use is not same as used */
        if (used)
        {
            /* not really 'used', just integrity data from there */
            nsv++;
            session->gpsdata.skyview[st].used = true;
        }
        st++;
    }

    session->gpsdata.satellites_visible = (int)st;
    session->gpsdata.satellites_used = (int)nsv;

    return SATELLITE_SET | USED_IS;
}

/*
 * Velocity Position ECEF message, UBX-NAV-VELECEF
 */
gps_mask_t
ubx_msg_nav_velecef(struct gps_device_t *session, unsigned char *buf,
                    size_t data_len)
{
    gps_mask_t mask = VECEF_SET;

    if (20 > data_len)
    {

        return 0;
    }

    session->iTOW = getleu32(buf, 0);
    session->gpsdata.fix.ecef.vx = getles32(buf, 4) / 100.0;
    session->gpsdata.fix.ecef.vy = getles32(buf, 8) / 100.0;
    session->gpsdata.fix.ecef.vz = getles32(buf, 12) / 100.0;
    session->gpsdata.fix.ecef.vAcc = getleu32(buf, 16) / 100.0;

    return mask;
}

/*
 * Velocity NED message, UBX-NAV-VELNED
 * protocol versions 15+
 */
gps_mask_t
ubx_msg_nav_velned(struct gps_device_t *session, unsigned char *buf,
                   size_t data_len)
{
    gps_mask_t mask = VNED_SET;

    if (36 > data_len)
    {

        return 0;
    }

    session->iTOW = getleu32(buf, 0);
    session->gpsdata.fix.NED.velN = getles32(buf, 4) / 100.0;
    session->gpsdata.fix.NED.velE = getles32(buf, 8) / 100.0;
    session->gpsdata.fix.NED.velD = getles32(buf, 12) / 100.0;
    return mask;
}

/*
 * SBAS Info UBX-NAV-SBAS
 * in u-blox 4_
 * in NEO-M9N
 * Not in some u-blox 9
 * Decode looks good, but data only goes to log.
 */
gps_mask_t
ubx_msg_nav_sbas(struct gps_device_t *session, unsigned char *buf,
                 size_t data_len)
{
    return 0;
}

/*
 * Multi-GNSS measurement Data -- UBX-RXM-MEASX
 * Not in u-blox 5, 6 or 7
 * u-blox 9, message version 0 (but no version byte!)
 * u-blox 9, message version 1
 */
gps_mask_t ubx_msg_rxm_measx(struct gps_device_t *session,
                             const unsigned char *buf,
                             size_t data_len)
{
    gps_mask_t mask = 0;
    unsigned int numMeas;
    int i;

    if (16 > data_len)
    {
        return 0;
    }

    numMeas = (unsigned int)getub(buf, 34);
    session->gpsdata.raw.avb_meas = numMeas;
    uint32_t towMS = getleu32(buf, 4);
    timespec_t ts_tow;
    MSTOTS(&ts_tow, towMS);
    session->gpsdata.raw.mtime = ts_tow;
    // zero out the structure as in the rawx case to make sure we see which ones changed
    log_trace("Got MEASX %d ", numMeas);
    if (numMeas > MAXCHANNELS)
    {
        return 0;
    }

    for (i = 0; i < numMeas; i++)
    {
        int off = 24 * i;
        short svId = (short)getub(buf, off + 45);
        log_trace("Sat SV: %d ", svId);
        /* code phase */
        session->gpsdata.raw.meas[svId].codephase = ((double)getleu32((const char *)buf, off + 60)) * 4.76837158e-7;
        session->gpsdata.skyview[svId].codephase = ((double)getleu32((const char *)buf, off + 60)) * 4.76837158e-7;
        session->gpsdata.raw.meas[svId].wholeChips = (getleu16((const char *)buf, off + 52));
        session->gpsdata.raw.meas[svId].fracChips = (getleu16((const char *)buf, off + 58));
        session->gpsdata.raw.meas[svId].doppler = ((double)getleu32((const char *)buf, off + 52)) * 0.2;
        log_trace("MESX SVID: %d CF: %f ", svId, session->gpsdata.raw.meas[svId].codephase);
    }

    return MEASX_SET;
}

/*
 * Multi-GNSS Raw measurement Data -- UBX-RXM-RAWX
 * Not in u-blox 5, 6 or 7
 * u-blox 9, message version 0 (but no version byte!)
 * u-blox 9, message version 1
 */
gps_mask_t ubx_msg_rxm_rawx(struct gps_device_t *session,
                            const unsigned char *buf,
                            size_t data_len)
{
    double rcvTow;
    uint16_t week;
    int8_t leapS;
    uint8_t numMeas;
    uint8_t recStat;
    uint8_t version;
    int i;
    const char *obs_code;
    timespec_t ts_tow;

    if (16 > data_len)
    {
        return 0;
    }

    /* Note: this is "approximately" GPS TOW, this is not iTOW */
    rcvTow = getled64((const char *)buf, 0); /* time of week in seconds */
    week = getleu16(buf, 8);
    leapS = getsb(buf, 10);
    numMeas = getub(buf, 11);

    // Save the number of measurements we have
    session->gpsdata.raw.avb_meas = numMeas;
    // printf("NMeas: %d", numMeas);
    recStat = getub(buf, 12);
    /* byte 13 is version on u-blox 9, reserved on u-blox 8
     * how is that supposed to work?? */
    // version = getub(buf, 13);

    /* convert GPS weeks and "approximately" GPS TOW to UTC */
    DTOTS(&ts_tow, rcvTow);
    // Do not set gpsdata.fix.time.  set gpsdata.raw.mtime
    // session->gpsdata.raw.mtime = gpsd_gpstime_resolv(session, week, ts_tow);

    /* zero the measurement data */
    /* so we can tell which meas never got set */
    if (numMeas > MAXCHANNELS)
    {
        return 0;
    }
    for (i = 0; i < numMeas; i++)
    {
        int off = 32 * i;
        /* pseudorange in meters */
        double prMes = (double)getled64((const char *)buf, off + 16);
        /* carrier phase in cycles */
        double cpMes = (double)getled64((const char *)buf, off + 24);
        /* doppler in Hz, positive towards sat */
        double doMes = (double) getlef32((const char *)buf, off + 32);
        uint8_t gnssId = getub(buf, off + 36);
        short svId = (short)getub(buf, off + 37);
        // printf("SVID %d\n", svId);
        //  reserved in u-blox 8, sigId in u-blox 9 (version 1)
        uint8_t sigId = getub(buf, off + 38);
        /* GLONASS frequency slot */
        uint8_t freqId = getub(buf, off + 39);
        /* carrier phase locktime in ms, max 64500ms */
        uint16_t locktime = getleu16(buf, off + 40);
        /* carrier-to-noise density ratio dB-Hz */
        uint8_t cno = getub(buf, off + 42);
        uint8_t prStdev = getub(buf, off + 43) & 0x0f;
        uint8_t cpStdev = getub(buf, off + 44) & 0x0f;
        uint8_t doStdev = getub(buf, off + 45) & 0x0f;
        /* tracking stat
         * bit 0 - prMes valid
         * bit 1 - cpMes valid
         * bit 2 - halfCycle valid
         * bit 3 - halfCycle subtracted from phase
         */
        uint8_t trkStat = 0x03;
        session->gpsdata.raw.meas[svId].gnssid = gnssId;
        session->gpsdata.raw.meas[svId].sigid = sigId;

        /* some of these are GUESSES as the u-blox codes do not
         * match RINEX codes */
        switch (gnssId)
        {
        case 0: /* GPS */
            switch (sigId)
            {
            default:
                /* let PPP figure it out */
            case 0: /* L1C/A */
                obs_code = "L1C";
                break;
            case 3: /* L2 CL */
                obs_code = "L2C";
                break;
            case 4: /* L2 CM */
                obs_code = "L2X";
                break;
            }
            break;
        case 1: /* SBAS */
            /* sigId added on protVer 27, and SBAS gone in protVer 27
             * so must be L1C/A */
            svId -= 100; /* adjust for RINEX 3 svid */

            obs_code = "L1C"; /* u-blox calls this L1C/A */
            /* SBAS can do L5I, but the code? */
            switch (sigId)
            {
            default:
                /* let PPP figure it out */
                break;
            case 0: /* L1C/A */
                obs_code = "L1C";
                break;
            }
            break;
        case 2: /* GALILEO */
            switch (sigId)
            {
            default:
                /* let PPP figure it out */
            case 0:               /*  */
                obs_code = "L1C"; /* u-blox calls this E1OS or E1C */
                break;
            case 1:               /*  */
                obs_code = "L1B"; /* u-blox calls this E1B */
                break;
            case 5:               /*  */
                obs_code = "L7I"; /* u-blox calls this E5bl */
                break;
            case 6:               /*  */
                obs_code = "L7Q"; /* u-blox calls this E5bQ */
                break;
            }
            break;
        case 3: /* BeiDou */
            switch (sigId)
            {
            default:
                /* let PPP figure it out */
            case 0:               /*  */
                obs_code = "L2Q"; /* u-blox calls this B1I D1 */
                break;
            case 1:               /*  */
                obs_code = "L2I"; /* u-blox calls this B1I D2 */
                break;
            case 2:               /*  */
                obs_code = "L7Q"; /* u-blox calls this B2I D1 */
                break;
            case 3:               /*  */
                obs_code = "L7I"; /* u-blox calls this B2I D2 */
                break;
            }
            break;
        default:           /* huh? */
        case 4:            /* IMES.  really? */
            obs_code = ""; /* u-blox calls this L1 */
            break;
        case 5: /* QZSS */
            switch (sigId)
            {
            default:
                /* let PPP figure it out */
            case 0:               /*  */
                obs_code = "L1C"; /* u-blox calls this L1C/A */
                break;
            case 4:               /*  */
                obs_code = "L2S"; /* u-blox calls this L2CM */
                break;
            case 5:               /*  */
                obs_code = "L2L"; /* u-blox calls this L2CL*/
                break;
            }
            break;
        case 6: /* GLONASS */
            switch (sigId)
            {
            default:
                /* let PPP figure it out */
            case 0:               /*  */
                obs_code = "L1C"; /* u-blox calls this L1OF */
                break;
            case 2:               /*  */
                obs_code = "L2C"; /* u-blox calls this L2OF */
                break;
            }
            break;
        }
        (void)strlcpy(session->gpsdata.raw.meas[i].obs_code, obs_code,
                      sizeof(session->gpsdata.raw.meas[i].obs_code));

        session->gpsdata.raw.meas[svId].svid = svId;
        session->gpsdata.raw.meas[svId].freqid = freqId;
        session->gpsdata.raw.meas[svId].snr = cno;
        session->gpsdata.raw.meas[svId].satstat = trkStat;
        if (trkStat & 1)
        {
            /* prMes valid */
            session->gpsdata.raw.meas[svId].pseudorange = prMes;
            session->gpsdata.skyview[svId].pseudorange = prMes;
        }
        else
        {
            session->gpsdata.raw.meas[svId].pseudorange = 0;
            session->gpsdata.skyview[svId].pseudorange = 0;
        }
        if ((trkStat & 2))
        {
            /* cpMes valid, RTKLIB uses 5 < cpStdev */
            session->gpsdata.raw.meas[svId].carrierphase = cpMes;
            session->gpsdata.skyview[svId].carrierphase = cpMes;
        }
        else
        {
            session->gpsdata.raw.meas[svId].carrierphase = 0;
            session->gpsdata.skyview[svId].carrierphase = 0;
        }
        session->gpsdata.raw.meas[svId].doppler = doMes;
        session->gpsdata.raw.meas[svId].deltarange = 0;
        session->gpsdata.raw.meas[svId].locktime = locktime;
        if (0 == locktime)
        {
            /* possible slip */
            session->gpsdata.raw.meas[svId].lli = 2;
        }
    }

    return RAW_SET;
}

/*
 * Raw Subframes - UBX-RXM-SFRB
 * In u-blox 7, only in raw firmware option
 * Not in u-blox 8 or 9
 */
gps_mask_t ubx_msg_rxm_sfrb(struct gps_device_t *session,
                            unsigned char *buf, size_t data_len)
{
    unsigned int i, chan, svid;
    uint32_t words[10];

    if (42 > data_len)
    {
        log_debug("UBX-RXM-SFRB message, runt payload len %zd", data_len);
        return 0;
    }

    chan = (unsigned int)getub(buf, 0);
    svid = (unsigned int)getub(buf, 1);
    log_debug("UBX-RXM-SFRB: %u %u", chan, svid);

    /* UBX does all the parity checking, but still bad data gets through */
    for (i = 0; i < 10; i++)
    {
        // bits 24 to 31 undefined, remove them.
        words[i] = (uint32_t)getleu32(buf, 4 * i + 2) & 0x00ffffff;
    }

    // probably GPS, could be SBAS
    return gpsd_interpret_subframe(session, GNSSID_GPS, svid, words);
}

/*
 * Raw Subframes - UBX-RXM-SFRBX
 * in u-blox 8, protver 17 and up, time sync firmware only
 * in u-blox F9P abd HPG only
 * not present  before u-blox8
 */
gps_mask_t ubx_msg_rxm_sfrbx(struct gps_device_t *session,
                             unsigned char *buf, size_t data_len)
{
    unsigned i;
    uint8_t gnssId, svId, freqId, numWords, chn, version;
    uint32_t words[17];
    char *chn_s;

    if (8 > data_len)
    {

        return 0;
    }

    numWords = getub(buf, 4);
    if (data_len != (size_t)(8 + (4 * numWords)) ||
        16 < numWords)
    {
        return 0;
    }

    gnssId = getub(buf, 0);
    svId = getub(buf, 1);
    freqId = getub(buf, 2);
    version = getub(buf, 6);
    chn = getub(buf, 5);
    if (1 < version)
    {
        // receiver channel in version 2 and up.
        // valid range 0 to 13?
        chn_s = "chn";
    }
    else
    {
        chn_s = "reserved";
    }

    if (0 == version)
    {
        // unknown ersion
        return 0;
    }

    memset(words, 0, sizeof(words));
    for (i = 0; i < numWords; i++)
    {
        // grab the words, don't mangle them
        words[i] = (uint32_t)getleu32(buf, 4 * i + 8);
    }

    // do we need freqId or chn?
    return gpsd_interpret_subframe_raw(session, gnssId, svId, words, numWords);
}

/**
 * SV Status Info
 *
 * May be good cycle ender
 *
 * Present in u-blox 7
 */
gps_mask_t
ubx_msg_rxm_svsi(struct gps_device_t *session, unsigned char *buf,
                 size_t data_len)
{
    return 0;
}

/* UBX-INF-* */
gps_mask_t
ubx_msg_inf(struct gps_device_t *session, unsigned char *buf, size_t data_len)
{
    return 0;
}

/**
 * Survey-in data - UBX-TIM-SVIN
 * Time Sync products only
 */
gps_mask_t
ubx_msg_tim_svin(struct gps_device_t *session, unsigned char *buf,
                 size_t data_len)
{
    return 0;
}

/**
 * Time Pulse Timedata - UBX-TIM-TP
 */
gps_mask_t
ubx_msg_tim_tp(struct gps_device_t *session, unsigned char *buf,
               size_t data_len)
{
    timespec_t time = (timespec_t){0, 0};
    clock_gettime(CLOCK_REALTIME, &time);
    // log_warn("TP @ : %ld - %ld", time.tv_sec, time.tv_nsec);
    session->gpsdata.currTime = time;
    gps_mask_t mask = ONLINE_SET;
    uint32_t towMS;
    uint32_t towSubMS;
    int32_t qErr;
    uint16_t week;
    uint8_t flags;
    uint8_t refInfo;
    timespec_t ts_tow;

    //reliable cycle - empty all data
    memset(&session->gpsdata.skyview, '\0', sizeof(&session->gpsdata.skyview));
    memset(&session->gpsdata.raw, '\0', sizeof(&session->gpsdata.raw));
    session->gpsdata.satellites_visible = 0;

    if (16 > data_len)
    {
        return 0;
    }

    towMS = getleu32(buf, 0);
    // towSubMS always seems zero, which will match the PPS
    towSubMS = getleu32(buf, 4);
    qErr = getles32(buf, 8);
    week = getleu16(buf, 12);
    flags = buf[14];
    refInfo = buf[15];

    /* are we UTC, and towSubMs is zero? */
    if (3 == (flags & 0x03) &&
        0 == towSubMS)
    {

        // leap already added!?!?
        // int saved_leap = session->context->leap_seconds;
        // remove it!
        // session->context->leap_seconds = 0;

        /* good, save qErr and qErr_time */
        MSTOTS(&ts_tow, towMS);
        session->gpsdata.tpTime = ts_tow;
        // session->gpsdata.qErr_time = gpsd_gpstime_resolv(session, week, ts_tow);

        // restore leap
    }

    // /* cast for 32 bit compatibility */
    // log_debug("TIM-TP: towMS %lu, towSubMS %lu, qErr %ld week %u "
    //           "flags %#x, refInfo %#x\n",
    //           (unsigned long)towMS, (unsigned long)towSubMS, (long)qErr,
    //           week, flags, refInfo);
    return mask;
}

gps_mask_t ubx_parse(struct gps_device_t *session, unsigned char *buf,
                     size_t len)
{
    size_t data_len;
    unsigned short msgid;
    gps_mask_t mask = 0;

    // the packet at least contains a head long enough for an empty message
    if (UBX_PREFIX_LEN > len)
    {
        return 0;
    }

    // session->cycle_end_reliable = true;
    session->iTOW = -1; // set by decoder

    // extract message id and length
    msgid = (buf[2] << 8) | buf[3];
    data_len = (size_t)getleu16(buf, 4);

    switch (msgid)
    {
    case UBX_ACK_ACK:
        if (2 <= data_len)
        {
        }
        break;
    case UBX_ACK_NAK:
        if (2 <= data_len)
        {
        }
        break;
    case UBX_NAV_CLOCK:
        log_debug("UBX-NAV-CLOCK");
        // mask = ubx_msg_nav_clock(session, &buf[UBX_PREFIX_LEN], data_len);
    case UBX_NAV_DOP:
        log_debug("UBX-NAV_DOP");
        // DOP seems to be the last NAV sent in a cycle, unless NAV-EOE
        // mask = ubx_msg_nav_dop(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_PVT:
        log_debug("UBX-NAV-PVT");
        mask = ubx_msg_nav_pvt(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_RELPOSNED:
        log_debug("UBX-NAV-RELPOSNED");
        // mask = ubx_msg_nav_relposned(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_RESETODO:
        log_debug("UBX-NAV-RESETODO");
        break;
    case UBX_NAV_SAT:
        log_info("UBX-NAV-SAT");
        mask = ubx_msg_nav_sat(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_SBAS:
        log_debug("UBX-NAV-SBAS");
        // mask = ubx_msg_nav_sbas(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_SIG:
        log_debug("UBX-NAV-SIG");
        break;
    case UBX_NAV_SOL:
        /* UBX-NAV-SOL deprecated in u-blox 6, gone in u-blox 9 and 10.
         * Use UBX-NAV-PVT instead */
        log_debug("UBX-NAV-SOL");
        // mask = ubx_msg_nav_sol(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_STATUS:
        log_debug("UBX-RXM-STATUS");
        // mask = ubx_msg_nav_status(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_SVIN:
        log_debug("UBX-RXM-SVIN");
        // mask = ubx_msg_tim_svin(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_SVINFO:
        // UBX-NAV-SVINFO deprecated, use UBX-NAV-SAT instead
        log_debug("UBX-NAV-SVINFO");
        // mask = ubx_msg_nav_svinfo(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_TIMEBDS:
        log_debug("UBX-NAV-TIMEBDS");
        break;
    case UBX_NAV_TIMEGAL:
        // mask = ubx_msg_nav_timegal(session, &buf[UBX_PREFIX_LEN], data_len);
        log_debug("UBX-NAV-TIMEGAL");
        break;
    case UBX_NAV_TIMEGLO:
        log_debug("UBX-NAV-TIMEGLO");
        break;
    case UBX_NAV_TIMEGPS:
        log_info("UBX-NAV-TIMEGPS");
        mask = ubx_msg_nav_timegps(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_TIMELS:
        // mask = ubx_msg_nav_timels(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_TIMEQZSS:
        log_debug("UBX-NAV-TIMEQZSS");
        break;
    case UBX_NAV_TIMEUTC:
        mask = ubx_msg_nav_timeutc(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_RXM_ALM:
        log_debug("UBX-RXM-ALM");
        break;
    case UBX_RXM_EPH:
        log_debug("UBX-RXM-EPH");
        break;
    case UBX_RXM_MEASX:
        log_info("UBX-RXM-MEASX");
        mask = ubx_msg_rxm_measx(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_RXM_RAWX:
        log_info("UBX-RXM-RAWX");
        mask = ubx_msg_rxm_rawx(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_RXM_SFRB:
        log_info("UBX-RXM-SFRB");
        mask = ubx_msg_rxm_sfrb(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_RXM_SFRBX:
        log_info("UBX-RXM-SFRBX");
        mask = ubx_msg_rxm_sfrbx(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_TIM_DOSC:
        log_debug("UBX-TIM-DOSC");
        break;
    case UBX_TIM_SMEAS:
        log_debug("UBX-TIM-SMEAS");
        break;
    case UBX_TIM_TM:
        log_debug("UBX-TIM-TM");
        break;
    case UBX_TIM_TP:
        log_info("UBX-TIM-TP");
        mask = ubx_msg_tim_tp(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_TIM_TOS:
        log_debug("UBX-TIM-TOS");
        break;


    default:
        log_debug("No clue what happened - unkn packet");
    }
    return mask | ONLINE_SET;
}
