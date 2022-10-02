/*
 * This file is Copyright 2010 by the GPSD project
 * BSD terms apply: see the file COPYING in the distribution root for details
 */
#ifndef _GPSD_UBX_H_
#define _GPSD_UBX_H_

#include "gps.h"
#include "gpsd.h"

typedef enum
{
    UBX_CLASS_NAV = 0x01,   /**< Navigation */
    UBX_CLASS_RXM = 0x02,   /**< Receiver Manager */
    UBX_CLASS_INF = 0x04,   /**< Informative text messages */
    UBX_CLASS_ACK = 0x05,   /**< (Not) Acknowledges for cfg messages */
    UBX_CLASS_CFG = 0x06,   /**< Configuration requests */
    UBX_CLASS_UPD = 0x09,   /**< Firmware updates */
    UBX_CLASS_MON = 0x0a,   /**< System monitoring */
    UBX_CLASS_AID = 0x0b,   /**< AGPS (Deprecated) */
    UBX_CLASS_TIM = 0x0d,   /**< Time */
    UBX_CLASS_ESF = 0x10,   /**< External Sensor Fusion */
    UBX_CLASS_MGA = 0x13,   /**< Multi GNSS Assistance */
    UBX_CLASS_LOG = 0x21,   /**< Log */
    UBX_CLASS_SEC = 0x27,   /**< Security */
    UBX_CLASS_HNR = 0x28,   /**< High Rate Nav Results  */
    UBX_CLASS_NMEA = 0xf0,  /**< NMEA, for configuring */
    UBX_CLASS_RTCM3 = 0xf5, /**< RTCM3, for configuring */
} ubx_classes_t;

#define UBX_MSGID(cls_, id_) (((cls_) << 8) | (id_))

typedef enum
{
    UBX_ACK_ACK = UBX_MSGID(UBX_CLASS_ACK, 0x01),
    UBX_ACK_NAK = UBX_MSGID(UBX_CLASS_ACK, 0x00),

    // USB-AID- Deprecated
    UBX_AID_ALM = UBX_MSGID(UBX_CLASS_AID, 0x30),
    UBX_AID_AOP = UBX_MSGID(UBX_CLASS_AID, 0x33),
    UBX_AID_DATA = UBX_MSGID(UBX_CLASS_AID, 0x10),
    UBX_AID_EPH = UBX_MSGID(UBX_CLASS_AID, 0x31),
    UBX_AID_HUI = UBX_MSGID(UBX_CLASS_AID, 0x02),
    UBX_AID_INI = UBX_MSGID(UBX_CLASS_AID, 0x01),
    UBX_AID_REQ = UBX_MSGID(UBX_CLASS_AID, 0x00),

    UBX_CFG_ANT = UBX_MSGID(UBX_CLASS_CFG, 0x13),
    UBX_CFG_CFG = UBX_MSGID(UBX_CLASS_CFG, 0x09),
    UBX_CFG_DAT = UBX_MSGID(UBX_CLASS_CFG, 0x06),
    UBX_CFG_DGNSS = UBX_MSGID(UBX_CLASS_CFG, 0x70),
    UBX_CFG_DOSC = UBX_MSGID(UBX_CLASS_CFG, 0x61),
    UBX_CFG_DYNSEED = UBX_MSGID(UBX_CLASS_CFG, 0x85),
    UBX_CFG_ESRC = UBX_MSGID(UBX_CLASS_CFG, 0x60),
    UBX_CFG_FIXSEED = UBX_MSGID(UBX_CLASS_CFG, 0x84),
    UBX_CFG_GEOFENCE = UBX_MSGID(UBX_CLASS_CFG, 0x69),
    UBX_CFG_GNSS = UBX_MSGID(UBX_CLASS_CFG, 0x3e),
    UBX_CFG_HNR = UBX_MSGID(UBX_CLASS_CFG, 0x5c),
    UBX_CFG_INF = UBX_MSGID(UBX_CLASS_CFG, 0x02),
    UBX_CFG_ITFM = UBX_MSGID(UBX_CLASS_CFG, 0x39),
    UBX_CFG_LOGFILTER = UBX_MSGID(UBX_CLASS_CFG, 0x47),
    UBX_CFG_MSG = UBX_MSGID(UBX_CLASS_CFG, 0x01),
    UBX_CFG_NAV5 = UBX_MSGID(UBX_CLASS_CFG, 0x24),
    UBX_CFG_NAVX5 = UBX_MSGID(UBX_CLASS_CFG, 0x23),
    UBX_CFG_NMEA = UBX_MSGID(UBX_CLASS_CFG, 0x17),
    UBX_CFG_ODO = UBX_MSGID(UBX_CLASS_CFG, 0x1e),
    UBX_CFG_PM2 = UBX_MSGID(UBX_CLASS_CFG, 0x3b),
    UBX_CFG_PMS = UBX_MSGID(UBX_CLASS_CFG, 0x86),
    UBX_CFG_PRT = UBX_MSGID(UBX_CLASS_CFG, 0x00),
    UBX_CFG_PWR = UBX_MSGID(UBX_CLASS_CFG, 0x57),
    UBX_CFG_RATE = UBX_MSGID(UBX_CLASS_CFG, 0x08),
    UBX_CFG_RINV = UBX_MSGID(UBX_CLASS_CFG, 0x34),
    UBX_CFG_RST = UBX_MSGID(UBX_CLASS_CFG, 0x04),
    UBX_CFG_RXM = UBX_MSGID(UBX_CLASS_CFG, 0x11),
    UBX_CFG_SBAS = UBX_MSGID(UBX_CLASS_CFG, 0x16),
    UBX_CFG_SMGR = UBX_MSGID(UBX_CLASS_CFG, 0x62),
    UBX_CFG_TMODE2 = UBX_MSGID(UBX_CLASS_CFG, 0x3D),
    UBX_CFG_TMODE3 = UBX_MSGID(UBX_CLASS_CFG, 0x71),
    UBX_CFG_TP5 = UBX_MSGID(UBX_CLASS_CFG, 0x31),
    UBX_CFG_TXSLOT = UBX_MSGID(UBX_CLASS_CFG, 0x53),
    UBX_CFG_USB = UBX_MSGID(UBX_CLASS_CFG, 0x1b),

    UBX_ESF_ALG = UBX_MSGID(UBX_CLASS_ESF, 0x14),
    UBX_ESF_INS = UBX_MSGID(UBX_CLASS_ESF, 0x15),
    UBX_ESF_MEAS = UBX_MSGID(UBX_CLASS_ESF, 0x02),
    UBX_ESF_RAW = UBX_MSGID(UBX_CLASS_ESF, 0x03),
    UBX_ESF_STATUS = UBX_MSGID(UBX_CLASS_ESF, 0x10),

    UBX_HNR_ATT = UBX_MSGID(UBX_CLASS_HNR, 0x01),
    UBX_HNR_INS = UBX_MSGID(UBX_CLASS_HNR, 0x02),
    UBX_HNR_PVT = UBX_MSGID(UBX_CLASS_HNR, 0x00),

    UBX_INF_DEBUG = UBX_MSGID(UBX_CLASS_INF, 0x04),
    UBX_INF_ERROR = UBX_MSGID(UBX_CLASS_INF, 0X00),
    UBX_INF_NOTICE = UBX_MSGID(UBX_CLASS_INF, 0x02),
    UBX_INF_TEST = UBX_MSGID(UBX_CLASS_INF, 0x03),
    /* where is UBX-INF-USER documented? */
    UBX_INF_USER = UBX_MSGID(UBX_CLASS_INF, 0x07),
    UBX_INF_WARNING = UBX_MSGID(UBX_CLASS_INF, 0X01),

    UBX_LOG_BATCH = UBX_MSGID(UBX_CLASS_LOG, 0x11),
    UBX_LOG_CREATE = UBX_MSGID(UBX_CLASS_LOG, 0x07),
    UBX_LOG_ERASE = UBX_MSGID(UBX_CLASS_LOG, 0x03),
    UBX_LOG_FINDTIME = UBX_MSGID(UBX_CLASS_LOG, 0x0e),
    UBX_LOG_INFO = UBX_MSGID(UBX_CLASS_LOG, 0x08),
    UBX_LOG_RETRIEVEBATCH = UBX_MSGID(UBX_CLASS_LOG, 0x10),
    UBX_LOG_RETRIEVEPOSEXTRA = UBX_MSGID(UBX_CLASS_LOG, 0x0f),
    UBX_LOG_RETRIEVEPOS = UBX_MSGID(UBX_CLASS_LOG, 0x0b),
    UBX_LOG_RETRIEVESTRING = UBX_MSGID(UBX_CLASS_LOG, 0x0d),
    UBX_LOG_RETRIEVE = UBX_MSGID(UBX_CLASS_LOG, 0x09),
    UBX_LOG_STRING = UBX_MSGID(UBX_CLASS_LOG, 0x04),

    UBX_MGA_ACK = UBX_MSGID(UBX_CLASS_MGA, 0x60),
    UBX_MGA_ANO = UBX_MSGID(UBX_CLASS_MGA, 0x20),
    UBX_MGA_BDS = UBX_MSGID(UBX_CLASS_MGA, 0x03),
    UBX_MGA_DBD = UBX_MSGID(UBX_CLASS_MGA, 0x80),
    UBX_MGA_FLASH = UBX_MSGID(UBX_CLASS_MGA, 0x21),
    UBX_MGA_GAL = UBX_MSGID(UBX_CLASS_MGA, 0x02),
    UBX_MGA_GLO = UBX_MSGID(UBX_CLASS_MGA, 0x06),
    UBX_MGA_GPS = UBX_MSGID(UBX_CLASS_MGA, 0x00),
    UBX_MGA_INI = UBX_MSGID(UBX_CLASS_MGA, 0x40),
    UBX_MGA_QZSS = UBX_MSGID(UBX_CLASS_MGA, 0x05),

    UBX_MON_BATCH = UBX_MSGID(UBX_CLASS_MON, 0x32),
    UBX_MON_EXCEPT = UBX_MSGID(UBX_CLASS_MON, 0x05),
    UBX_MON_GNSS = UBX_MSGID(UBX_CLASS_MON, 0x28),
    UBX_MON_HW2 = UBX_MSGID(UBX_CLASS_MON, 0x0b),
    UBX_MON_HW3 = UBX_MSGID(UBX_CLASS_MON, 0x37),
    UBX_MON_HW = UBX_MSGID(UBX_CLASS_MON, 0x09),
    UBX_MON_IO = UBX_MSGID(UBX_CLASS_MON, 0x02),
    UBX_MON_IPC = UBX_MSGID(UBX_CLASS_MON, 0x03),
    UBX_MON_MSGPP = UBX_MSGID(UBX_CLASS_MON, 0x06),
    UBX_MON_PATCH = UBX_MSGID(UBX_CLASS_MON, 0x27),
    UBX_MON_RF = UBX_MSGID(UBX_CLASS_MON, 0x38),
    UBX_MON_RXBUF = UBX_MSGID(UBX_CLASS_MON, 0x07),
    UBX_MON_RXR = UBX_MSGID(UBX_CLASS_MON, 0x21),
    UBX_MON_SCHED = UBX_MSGID(UBX_CLASS_MON, 0x01),
    UBX_MON_SMGR = UBX_MSGID(UBX_CLASS_MON, 0x2e),
    UBX_MON_SPAN = UBX_MSGID(UBX_CLASS_MON, 0x31),
    UBX_MON_TXBUF = UBX_MSGID(UBX_CLASS_MON, 0x08),
    UBX_MON_USB = UBX_MSGID(UBX_CLASS_MON, 0x0a),
    UBX_MON_VER = UBX_MSGID(UBX_CLASS_MON, 0x04),

    UBX_NAV_AOPSTATUS = UBX_MSGID(UBX_CLASS_NAV, 0x60),
    UBX_NAV_ATT = UBX_MSGID(UBX_CLASS_NAV, 0x05),
    UBX_NAV_CLOCK = UBX_MSGID(UBX_CLASS_NAV, 0x22),
    UBX_NAV_DGPS = UBX_MSGID(UBX_CLASS_NAV, 0x31),
    UBX_NAV_DOP = UBX_MSGID(UBX_CLASS_NAV, 0x04),
    UBX_NAV_EELL = UBX_MSGID(UBX_CLASS_NAV, 0x3d),
    UBX_NAV_EKFSTATUS = UBX_MSGID(UBX_CLASS_NAV, 0x40),
    UBX_NAV_EOE = UBX_MSGID(UBX_CLASS_NAV, 0x61),
    UBX_NAV_GEOFENCE = UBX_MSGID(UBX_CLASS_NAV, 0x39),
    UBX_NAV_HPPOSECEF = UBX_MSGID(UBX_CLASS_NAV, 0x13),
    UBX_NAV_HPPOSLLH = UBX_MSGID(UBX_CLASS_NAV, 0x14),
    UBX_NAV_ODO = UBX_MSGID(UBX_CLASS_NAV, 0x09),
    UBX_NAV_ORB = UBX_MSGID(UBX_CLASS_NAV, 0x34),
    UBX_NAV_POSECEF = UBX_MSGID(UBX_CLASS_NAV, 0x01),
    UBX_NAV_POSLLH = UBX_MSGID(UBX_CLASS_NAV, 0x02),
    UBX_NAV_POSUTM = UBX_MSGID(UBX_CLASS_NAV, 0x08),
    UBX_NAV_PVT = UBX_MSGID(UBX_CLASS_NAV, 0x07),
    UBX_NAV_RELPOSNED = UBX_MSGID(UBX_CLASS_NAV, 0x3c),
    UBX_NAV_RESETODO = UBX_MSGID(UBX_CLASS_NAV, 0x10),
    UBX_NAV_SAT = UBX_MSGID(UBX_CLASS_NAV, 0x35),
    UBX_NAV_SBAS = UBX_MSGID(UBX_CLASS_NAV, 0x32),
    UBX_NAV_SIG = UBX_MSGID(UBX_CLASS_NAV, 0x43),
    UBX_NAV_SOL = UBX_MSGID(UBX_CLASS_NAV, 0x06),
    UBX_NAV_STATUS = UBX_MSGID(UBX_CLASS_NAV, 0x03),
    UBX_NAV_SVINFO = UBX_MSGID(UBX_CLASS_NAV, 0x30),
    UBX_NAV_SVIN = UBX_MSGID(UBX_CLASS_NAV, 0x3b),
    UBX_NAV_TIMEBDS = UBX_MSGID(UBX_CLASS_NAV, 0x24),
    UBX_NAV_TIMEGAL = UBX_MSGID(UBX_CLASS_NAV, 0x25),
    UBX_NAV_TIMEGLO = UBX_MSGID(UBX_CLASS_NAV, 0x23),
    UBX_NAV_TIMEGPS = UBX_MSGID(UBX_CLASS_NAV, 0x20),
    UBX_NAV_TIMELS = UBX_MSGID(UBX_CLASS_NAV, 0x26),
    UBX_NAV_TIMEQZSS = UBX_MSGID(UBX_CLASS_NAV, 0x27),
    UBX_NAV_TIMEUTC = UBX_MSGID(UBX_CLASS_NAV, 0x21),
    UBX_NAV_VELECEF = UBX_MSGID(UBX_CLASS_NAV, 0x11),
    UBX_NAV_VELNED = UBX_MSGID(UBX_CLASS_NAV, 0x12),

    UBX_RXM_ALM = UBX_MSGID(UBX_CLASS_RXM, 0x30),
    UBX_RXM_EPH = UBX_MSGID(UBX_CLASS_RXM, 0x31),
    UBX_RXM_IMES = UBX_MSGID(UBX_CLASS_RXM, 0x61),
    UBX_RXM_MEASX = UBX_MSGID(UBX_CLASS_RXM, 0x14),
    UBX_RXM_PMREQ = UBX_MSGID(UBX_CLASS_RXM, 0x41),
    UBX_RXM_POSREQ = UBX_MSGID(UBX_CLASS_RXM, 0x40),
    UBX_RXM_RAW = UBX_MSGID(UBX_CLASS_RXM, 0x10),
    UBX_RXM_RAWX = UBX_MSGID(UBX_CLASS_RXM, 0x15),
    UBX_RXM_RLM = UBX_MSGID(UBX_CLASS_RXM, 0x59),
    UBX_RXM_RTCM = UBX_MSGID(UBX_CLASS_RXM, 0x32),
    UBX_RXM_SFRB = UBX_MSGID(UBX_CLASS_RXM, 0x11),
    UBX_RXM_SFRBX = UBX_MSGID(UBX_CLASS_RXM, 0x13),
    UBX_RXM_SVSI = UBX_MSGID(UBX_CLASS_RXM, 0x20),

    UBX_SEC_SIGN = UBX_MSGID(UBX_CLASS_SEC, 0x01),
    UBX_SEC_UNIQID = UBX_MSGID(UBX_CLASS_SEC, 0x03),
    // UBX_SEC_SESSID      = UBX_MSGID(UBX_CLASS_SEC, 0x),  // Undocumented

    UBX_TIM_DOSC = UBX_MSGID(UBX_CLASS_TIM, 0x11),
    UBX_TIM_FCHG = UBX_MSGID(UBX_CLASS_TIM, 0x16),
    UBX_TIM_HOC = UBX_MSGID(UBX_CLASS_TIM, 0x17),
    UBX_TIM_SMEAS = UBX_MSGID(UBX_CLASS_TIM, 0x13),
    UBX_TIM_SVIN = UBX_MSGID(UBX_CLASS_TIM, 0x04),
    UBX_TIM_TM2 = UBX_MSGID(UBX_CLASS_TIM, 0x03),
    UBX_TIM_TM = UBX_MSGID(UBX_CLASS_TIM, 0x02),
    UBX_TIM_TOS = UBX_MSGID(UBX_CLASS_TIM, 0x12),
    UBX_TIM_TP = UBX_MSGID(UBX_CLASS_TIM, 0x01),
    UBX_TIM_VCOCAL = UBX_MSGID(UBX_CLASS_TIM, 0x15),
    UBX_TIM_VRFY = UBX_MSGID(UBX_CLASS_TIM, 0x06),

    UBX_UPD_DOWNL = UBX_MSGID(UBX_CLASS_UPD, 0x01),
    UBX_UPD_EXEC = UBX_MSGID(UBX_CLASS_UPD, 0x03),
    UBX_UPD_MEMCPY = UBX_MSGID(UBX_CLASS_UPD, 0x04),
    UBX_UPD_SOS = UBX_MSGID(UBX_CLASS_UPD, 0x14),
    UBX_UPD_UPLOAD = UBX_MSGID(UBX_CLASS_UPD, 0x02),

} ubx_message_t;

typedef enum
{
    UBX_MODE_NOFIX = 0x00,  /* no fix available */
    UBX_MODE_DR = 0x01,     /* Dead reckoning */
    UBX_MODE_2D = 0x02,     /* 2D fix */
    UBX_MODE_3D = 0x03,     /* 3D fix */
    UBX_MODE_GPSDR = 0x04,  /* GPS + dead reckoning */
    UBX_MODE_TMONLY = 0x05, /* Time-only fix */
} ubx_mode_t;

#define UBX_LOG_BATCH_VALID_DATE 0x01
#define UBX_LOG_BATCH_VALID_TIME 0x02
#define UBX_LOG_BATCH_VALID_DATE_TIME (UBX_LOG_BATCH_VALID_DATE | \
                                       UBX_LOG_BATCH_VALID_TIME)
#define UBX_LOG_BATCH_CONTENTVALID_EXTRA_PVT 0x01
#define UBX_LOG_BATCH_CONTENTVALID_EXTRA_ODO 0x02

#define UBX_SOL_FLAG_GPS_FIX_OK 0x01
#define UBX_SOL_FLAG_DGPS 0x02
#define UBX_SOL_VALID_WEEK 0x04
#define UBX_SOL_VALID_TIME 0x08

#define UBX_TIMEGPS_VALID_TIME 0x01
#define UBX_TIMEGPS_VALID_WEEK 0x02
#define UBX_TIMEGPS_VALID_LEAP_SECOND 0x04

#define UBX_TIMEGAL_VALID_TIME 0x01
#define UBX_TIMEGAL_VALID_WEEK 0x02
#define UBX_TIMEGAL_VALID_LEAP_SECOND 0x04

/* from UBX_NAV_SVINFO */
#define UBX_SAT_USED 0x01
#define UBX_SAT_DGPS 0x02
#define UBX_SAT_EPHALM 0x04
#define UBX_SAT_EPHEM 0x08
#define UBX_SAT_UNHEALTHY 0x10

#define UBX_SIG_IDLE 0
#define UBX_SIG_SRCH1 1
#define UBX_SIG_SRCH2 2
#define UBX_SIG_DETECT 3
#define UBX_SIG_CDLK 4
#define UBX_SIG_CDCRLK1 5
#define UBX_SIG_CDCRLK2 6
#define UBX_SIG_NAVMSG 7

#define UBX_NAV_PVT_VALID_DATE 0x01
#define UBX_NAV_PVT_VALID_TIME 0x02
#define UBX_NAV_PVT_VALID_RESL 0x04
#define UBX_NAV_PVT_VALID_DATE_TIME (UBX_NAV_PVT_VALID_DATE | \
                                     UBX_NAV_PVT_VALID_TIME)
#define UBX_NAV_PVT_VALID_MAG 0x08

#define UBX_NAV_PVT_FLAG_GPS_FIX_OK 0x01
#define UBX_NAV_PVT_FLAG_DGPS 0x02
#define UBX_NAV_PVT_FLAG_HDG_OK 0x20

#define POW2_M5  0.03125
#define POW2_M19 1.907348632812500e-6
#define POW2_M29 1.862645149230957e-9
#define POW2_M31 4.656612873077393e-10
#define POW2_M33 1.164153218269348e-10
#define POW2_M43 1.136868377216160e-13
#define POW2_M55 2.775557561562891e-17

#define POW2_M50 8.881784197001252e-016
#define POW2_M30 9.313225746154785e-010
#define POW2_M27 7.450580596923828e-009
#define POW2_M24 5.960464477539063e-008

gps_mask_t ubx_parse(struct gps_device_t *session, unsigned char *buf,
                     size_t len);
gps_mask_t ubx_msg_log_batch(struct gps_device_t *session,
                             unsigned char *buf, size_t data_len);
gps_mask_t ubx_msg_log_info(struct gps_device_t *session,
                            unsigned char *buf, size_t data_len);
gps_mask_t ubx_msg_log_retrievepos(struct gps_device_t *session,
                                   unsigned char *buf, size_t data_len);
gps_mask_t ubx_msg_log_retrieveposextra(struct gps_device_t *session,
                                        unsigned char *buf,
                                        size_t data_len);
gps_mask_t ubx_msg_log_retrievestring(struct gps_device_t *session,
                                      unsigned char *buf,
                                      size_t data_len);
gps_mask_t ubx_msg_nav_dop(struct gps_device_t *session,
                           unsigned char *buf, size_t data_len);
gps_mask_t ubx_msg_nav_eoe(struct gps_device_t *session,
                           unsigned char *buf, size_t data_len);
gps_mask_t ubx_msg_inf(struct gps_device_t *session, unsigned char *buf,
                       size_t data_len);
gps_mask_t ubx_msg_nav_posecef(struct gps_device_t *session,
                               unsigned char *buf, size_t data_len);
gps_mask_t ubx_msg_nav_pvt(struct gps_device_t *session,
                           unsigned char *buf, size_t data_len);
gps_mask_t ubx_msg_mon_ver(struct gps_device_t *session,
                           unsigned char *buf, size_t data_len);
gps_mask_t ubx_msg_nav_sat(struct gps_device_t *session,
                           unsigned char *buf, size_t data_len);
gps_mask_t ubx_msg_nav_sol(struct gps_device_t *session,
                           unsigned char *buf, size_t data_len);
gps_mask_t ubx_msg_nav_svinfo(struct gps_device_t *session,
                              unsigned char *buf, size_t data_len);
gps_mask_t ubx_msg_nav_timegps(struct gps_device_t *session,
                               unsigned char *buf, size_t data_len);
gps_mask_t ubx_msg_nav_velecef(struct gps_device_t *session,
                               unsigned char *buf, size_t data_len);
gps_mask_t ubx_msg_nav_sbas(struct gps_device_t *session,
                            unsigned char *buf, size_t data_len);
gps_mask_t ubx_msg_tim_tp(struct gps_device_t *session,
                          unsigned char *buf, size_t data_len);
gps_mask_t ubx_msg_nav_clock(struct gps_device_t *session, unsigned char *buf,
                             size_t data_len);
void ubx_mode(struct gps_device_t *session, int mode);

extern size_t strlcpy(char *dst, const char *src, size_t siz);


#endif /* _GPSD_UBX_H_ */
// vim: set expandtab shiftwidth=4
