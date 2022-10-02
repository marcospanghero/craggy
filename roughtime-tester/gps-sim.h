/**
 * multi-sdr-gps-sim generates a IQ data stream on-the-fly to simulate a
 * GPS L1 baseband signal using a SDR platform like HackRF or ADLAM-Pluto.
 *
 * This file is part of the Github project at
 * https://github.com/mictronics/multi-sdr-gps-sim.git
 *
 * Copyright © 2021 Mictronics
 * Distributed under the MIT License.
 *
 */

#ifndef GPS_SIM_H
#define GPS_SIM_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <pthread.h>
#include <stdatomic.h>

#include "serial_api/gpsd.h"
#include "serial_api/gps.h"

#include "gps-core.h"

#define PRINT_MSG(f_, ...) printf((f_ "\n"), ##__VA_ARGS__)

#define NOTUSED(V) ((void)V)

// Sampling data format
#define SC08 sizeof(signed char)
#define SC16 sizeof(signed short)



/* Simulator location. */
typedef struct
{
    double lat;    // Latitude
    double lon;    // Longitude
    double height; // Height/Elevation
    gpstime_t start;
} location_t;

struct fixsource_t
/* describe a data source */
{
    char *spec; /* pointer to actual storage */
    char *server;
    char *port;
    char *device;
};

/* All the GPS simulators variables. */
typedef struct
{

    char port_name[256];

    atomic_bool main_exit;
    atomic_bool gps_thread_exit;
    atomic_bool gps_thread_running;
    atomic_bool gps_serial_thread_exit;
    atomic_bool gps_serial_thread_running;

    pid_t main_thread;
    pid_t serial_thread;
    pid_t gps_core_thread;

   
    bool sat_simulated[32];
    bool sat_in_view[32];    

    timespec_t compensation;
    pthread_cond_t gps_init_done; // Condition signals GPS thread is running
    pthread_mutex_t gps_serial_lock;
    pthread_t gps_serial_thread;
    pthread_cond_t gps_serial_init_done; // Condition signals GPS thread is running
    location_t location;                 // Simulator geo location
    struct gps_device_t gpsdata;
    bool external_data_ready;
    bool pre_synch;
    bool synch;
    bool external;
    bool raw_set;
    bool skyview_set;


} simulator_t;

void set_thread_name(const char *name);
int thread_to_core(int core_id);

#endif /* GPS_SIM_H */
