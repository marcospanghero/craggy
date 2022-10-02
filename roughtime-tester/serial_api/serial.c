/*H**********************************************************************
 * FILENAME :        serial.c             DESIGN REF: GPS-SYNCHROSIM
 *
 * DESCRIPTION :
 *       Serial port routines.
 *
 * PUBLIC FUNCTIONS :
 *       void *gps_serial_thread_ep(void *arg)
 *
 * NOTES :
 *       This function handles the serial port thread. 
 *       This handles the data incoming from a reference receiver
 *       Requires UBLOX receiver (M8T, F9P or above)
 *       Requires NAV-PVT, RXM-MEASX, RXM-RAWX, TIM-TP messages     
 *       Copyright A.N.Other Co. 1990, 1995.  All rights reserved.
 *
 * AUTHOR :    Marco Spanghero       
 *
 * CHANGES :
 *       Fixed data buffers behavior. Now reset all every time we get messages
 *
 * REF NO  VERSION DATE    WHO     DETAIL
 * F21/33  A.03.04 22Jan99 JR      Function CalcHuffman corrected
 *
 *H*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <pthread.h>

#include <unistd.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#include <sys/time.h>

#include "serial-driver.h"
#include "driver_ubx.h"
#include "gpsd.h"

#include <signal.h>
#include "../others/log.h"
#include "../gps-sim.h"
#include "serial.h"
#include "../others/log.h"

/*
* Define how many full cycles we want to wait untill we are pretty sure the GPS data we have is decent
*/

#define BACKOFF 6
/*
* Which serial port to connect to
* TODO: move this to a config file so we don't have to recompile all the time
*/
serial_port_t serial_port;

uint8_t ret = 0;

/*
* Holds all the svIDs (It is not used actually)
* TODO: remove
*/
uint8_t svIDs[MAX_CHAN];

/*!
 * Utility function: prints all the buffer as a string of hex bytes. Great for parsing and checking
 */

void print_hex(unsigned char *s, int len)
{
    int x = 0;
    while (x <= len)
    {
        log_debug("%02x ", (unsigned char)*s++);
        x++;
    }
    log_debug("\n");
}

/*!
 * Thread main function
 */

void *gps_serial_thread_ep(void *arg)
{
    log_set_level(LOG_INFO);
    bool run = true;
    thread_to_core(2);
    
    // Main simulator object
    simulator_t *simulator = (simulator_t *)(arg);

    /*!
     * Open serial port -> configs are in the serial port driver
     * TODO: move into configuration file externally or at least in the header file
     * Possibly we should get these from the main thread object, by creating the serial object there
     *  */ 
    ret = serial_port_init_port(simulator->port_name, &serial_port);
    ret = serial_port_open_port(&serial_port);

/*!
 * Bunch of different params used for reading packets
 * avb: data we get avb at the serial port
 * multi_fragment_offset: position in the frame for packets that are made of multiple fragments
 */
    size_t avb = 0;
    size_t multi_fragment_offset = 0;
    size_t packet_len = 0;
    size_t expected_len = 0;

    timespec_t actual_time = (timespec_t){0, 0};
    timespec_t rx_time = (timespec_t){0, 0};
    timespec_t rx_time_start = (timespec_t){0, 0};
    timespec_t rx_time_stop = (timespec_t){0, 0};
    timespec_t rx_time_delta = (timespec_t){0, 0};
    timespec_t rx_time_total = (timespec_t){0, 0};

    struct gps_device_t device;
    gps_mask_t mask = 0;

    // PPS structures
    int num;
    int avail_mode;
    int i = 0;
    int ret;

    memset(&device, 0, sizeof(struct gps_device_t));
    gpsd_zero_satellites(&device.gpsdata);
    gpsd_zero_raw(&device.gpsdata);

    unsigned char msg_buf[1024];
    unsigned char rx_buf[1024];
    memset(msg_buf, 0, 1024);
    memset(rx_buf, 0, 1024);
    bool sent_once = false;
    bool sent_raw = false;
    bool sent_skyview = false;

    int count_raw = 0;
    int count_measx = 0;

    // initialize things for the packet parser

    // setup the 1pps source
    struct timespec timeout = {2, 0};

    int count = 0;

    do
    {
        if (simulator->gps_serial_thread_running == false)
        {
            simulator->gps_serial_thread_running = true;
            pthread_cond_signal(&(simulator->gps_serial_init_done));
            log_info("Started gps loop");
        }
        ioctl(serial_port.port_descriptor, FIONREAD, &avb);
        if (avb > 0)
        {            // print_hex(rx_buf, ret);
            if (rx_buf[0] == 0xb5 && rx_buf[1] == 0x62)
            {
                rx_time = rx_time_start;
                expected_len = rx_buf[4] | rx_buf[5] << 8;
                log_trace("RX Fragment %d %02x%02x EXP Len: %ld Read time: [%ld.%ld]", ret, rx_buf[0], rx_buf[1], expected_len, rx_time_delta.tv_sec, rx_time_delta.tv_nsec);

                multi_fragment_offset = 0;
                packet_len = 0;
            }
            else
            {
                multi_fragment_offset = multi_fragment_offset + ret;
                log_trace("Multifragment packet: %d", multi_fragment_offset);
            }
            packet_len += ret;
            memcpy(&msg_buf[multi_fragment_offset], &rx_buf, ret);
            memset(rx_buf, 0, 1024);

            if (packet_len == expected_len + 8)
            {
                log_trace("[%ld.%ld] [%d] MSG FULL Len: %ld %x%x ", rx_time.tv_sec, rx_time.tv_nsec, device.gpsdata.subframe.subframe_num, packet_len, msg_buf[0], msg_buf[1]);
                timespec_sub(&rx_time, &rx_time, &rx_time_total);
                mask = ubx_parse(&device, msg_buf, packet_len);
                memset(msg_buf, 0, 1024);
                // wait until we have the first raw meas observation and we have a valid lock. After that, we can start sending stuff to the main driver. After we updated the initial skyview, we send out the subframe info (up to 30s)

                // What we want to collect from the raw measuremnts is:
                // - Initial set of satellites
                // - Initial code offset
                // - Initial carrier offset

                multi_fragment_offset = 0;
                packet_len = 0;
                simulator->gpsdata = device;
            }
        }
    } while (simulator->gps_serial_thread_exit == false);

end_gps_thread:
    printf("Exit Serial thread\n");
    serial_port_close(serial_port.port_descriptor);
    simulator->gps_serial_thread_exit = true;
    pthread_cond_signal(&(simulator->gps_serial_init_done));
    pthread_exit(NULL);
}