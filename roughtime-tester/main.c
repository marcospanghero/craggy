/*

What this should do: every time we detect a GNSS update we should poll the roughtime server. This will gives us a baseline trusted time, until we have a lock in the GNSS

Requirements: STANDARD POSIX code!! no wierd linux magic, no wierd mcu stuff.

1) how do we get precise timing? This is not a crazy important problem, the main issue is that we want to make sure that the first time point we get from the GPS makes more or less sense (UTC wise)
2) Additionally, we can use the roughtime authenticated time to

*/

#include <string.h>
#include <time.h>
#include <stdio.h>
#include <inttypes.h>
#include <getopt.h>
#include <assert.h>
#include <signal.h>

#include <pthread.h>

#include <unistd.h>
#include <fcntl.h>

#include "base64.h"
#include "CraggyTransport.h"
#include "CraggyClient.h"

#include "serial_api/serial.h"
#include "others/log.h"
#include "gps-sim.h"
#include "gps-core.h"

int run = 1;

simulator_t simulator;

static u_int64_t TimeUs(clockid_t clock)
{
    struct timespec tv;
    if (clock_gettime(clock, &tv))
    {
        abort();
    }
    u_int64_t ret = tv.tv_sec;
    ret *= 1000000;
    ret += tv.tv_nsec / 1000;
    return ret;
}

// MonotonicUs returns the value of the monotonic clock in microseconds.
u_int64_t MonotonicUs() { return TimeUs(CLOCK_MONOTONIC); }

// MonotonicUs returns the value of the realtime clock in microseconds.
u_int64_t RealtimeUs() { return TimeUs(CLOCK_REALTIME); }

static void simulator_init(void)
{
    simulator.main_exit = false;

    simulator.location.lat = 0;
    simulator.location.lon = 0;
    simulator.location.height = 0;

    simulator.external = false;
    simulator.external_data_ready = false;
    simulator.raw_set = false;
    simulator.skyview_set = false;

    simulator.pre_synch = false;
    simulator.synch = false;

    memset(&(simulator.gpsdata), 0, sizeof(struct gps_device_t));
    pthread_cond_init(&simulator.gps_serial_init_done, NULL);
    pthread_mutex_init(&simulator.gps_serial_lock, NULL);
}

void sig_handler(int sig)
{
    if (sig == SIGINT)
    {
        printf("GPS parsing stopped by SIGINT\n");
        run = -1;
    }
}

int main(int argc, char *argv[])
{
    signal(SIGINT, sig_handler);
    CraggyResult craggyResult;

    struct timespec timeout;

    int result = 1;

    static struct option long_options[] = {
        {"host", required_argument, 0, 'h'},
        {"key", required_argument, 0, 'k'},
        {"nonce", optional_argument, 0, 'n'},
        {"intervals", optional_argument, 0, 'i'},
        {"repreats", optional_argument, 0, 'r'},
        {"gpsport", optional_argument, 0, 'p'},
        {0, 0, 0, 0}};

    int c;

    char *hostname = NULL;
    char *nonce = NULL;
    char *publicKey = NULL;
    char *gpsPort = NULL;
    uint8_t repeats = 1;
    uint8_t intervals = 1;

    char byte;
    int avb = 0;

    while (1)
    {

        int option_index = 0;
        c = getopt_long(argc, argv, "h:k:n:i:r:p:", long_options, &option_index);

        /* Detect the end of the options. */
        if (c == -1)
            break;

        switch (c)
        {
        case 0:
            /* If this option set a flag, do nothing else now. */
            if (long_options[option_index].flag != 0)
                break;
            printf("option %s", long_options[option_index].name);
            if (optarg)
                printf(" with arg %s", optarg);
            printf("\n");
            break;

        case 'h':
            hostname = malloc(strlen(optarg) + 1);
            hostname = strcpy(hostname, optarg);
            break;

        case 'n':
            nonce = malloc(strlen(optarg) + 1);
            nonce = strcpy(hostname, optarg);
            break;

        case 'k':
            publicKey = malloc(strlen(optarg) + 1);
            publicKey = strcpy(publicKey, optarg);
            break;

        case 'p':
            gpsPort = strcpy(simulator.port_name, optarg);
            break;

        case 'i':
            intervals = atoi(optarg);
            printf("Will poll every %d seconds\n", intervals);
            break;

        case 'r':
            repeats = atoi(optarg);
            printf("Will poll for %d times\n", repeats);
            break;

        case '?':
            /* getopt_long already printed an error message. */
            break;

        default:
            abort();
        }
    }

    if (publicKey == NULL || hostname == NULL || gpsPort == NULL)
    {
        printf("usage: roughtimetester -h <hostname:port> -k <public key> -p </dev/gps>");
        return 1;
    }

    // Prepare to spawn thread for external receiver
    pthread_create(&simulator.gps_serial_thread, NULL, gps_serial_thread_ep, &simulator);
    pthread_mutex_lock(&(simulator.gps_serial_lock));
    int ret = pthread_cond_timedwait(&(simulator.gps_serial_init_done), &(simulator.gps_serial_lock), &timeout);
    pthread_mutex_unlock(&(simulator.gps_serial_lock));
    if (ret == ETIMEDOUT)
    {
        log_error("Time out waiting for External GPS thread. Running?");
    }
    else
    {
        log_info("Started External GPS thread - data is coming in now");
    }

    craggy_rough_time_public_key_t rootPublicKey;
    size_t base64DecodedRootPublicKeyLen = 0;
    unsigned char *base64DecodedRootPublicKey = base64_decode((const unsigned char *)publicKey, strlen(publicKey), &base64DecodedRootPublicKeyLen);

    if (base64DecodedRootPublicKeyLen != CRAGGY_ROUGH_TIME_PUBLIC_KEY_LENGTH)
    {
        printf("Public key length must be %d byte(s) (got %zu after base64 decoding)", CRAGGY_ROUGH_TIME_PUBLIC_KEY_LENGTH, base64DecodedRootPublicKeyLen);
        goto error;
    }
    memcpy(&rootPublicKey, base64DecodedRootPublicKey, CRAGGY_ROUGH_TIME_PUBLIC_KEY_LENGTH);
    free(base64DecodedRootPublicKey);

    craggy_rough_time_request_t requestBuf;
    memset(requestBuf, 0, sizeof(craggy_rough_time_request_t));

    craggy_rough_time_nonce_t nonceBytes;
    memset(nonceBytes, 1, CRAGGY_ROUGH_TIME_NONCE_LENGTH);

    if (nonce != NULL)
    {
        size_t outLen = 0;
        unsigned char *decodedNonceBytes = base64_decode((unsigned char *)nonce, strlen(nonce), &outLen);
        if (outLen != CRAGGY_ROUGH_TIME_NONCE_LENGTH)
        {
            printf("Nonce length must be %d byte(s) (got %zu after base64 decoding)", CRAGGY_ROUGH_TIME_NONCE_LENGTH, outLen);
            goto error;
        }
        memcpy(nonceBytes, decodedNonceBytes, outLen);
        free(decodedNonceBytes);
    }
    else
    {
        if (!craggy_generateNonce(&craggyResult, nonceBytes))
        {
            printf("Error generating nonce: %d", craggyResult);
            goto error;
        }
    }

    while (repeats > 0)
    {
        if (simulator.tp_lock)
        {

            if (craggy_createRequest(nonceBytes, requestBuf))
            {

                log_info("--------------- START ---------------");
                craggy_rough_time_t timestamp;
                uint32_t radius;
                const uint32_t start_us = MonotonicUs();

                size_t responseBufLen = CRAGGY_ROUGH_TIME_MIN_REQUEST_SIZE * 3;
                craggy_rough_time_response_t responseBuf[responseBufLen];

                if (craggy_makeRequest(hostname, requestBuf, &craggyResult, responseBuf, &responseBufLen))
                {

                    if (!craggy_processResponse(nonceBytes, rootPublicKey, responseBuf, responseBufLen, &craggyResult, &timestamp, &radius))
                    {
                        printf("Error parsing response: %d", craggyResult);
                        goto error;
                    }

                    // We assume that the path to the Roughtime server is symmetric and thus add
                    // half the round-trip time to the server's timestamp to produce our estimate
                    // of the current time.
                    const uint32_t end_us = MonotonicUs();
                    const uint32_t end_realtime_us = RealtimeUs();
                    timestamp += (end_us - start_us) / 2;

                    log_info("Craggy Timestamp: %ld", timestamp);
                    log_info("GPSTimestamp: %lf", (simulator.gpsdata.gpsdata.fix.time.tv_sec + simulator.gpsdata.gpsdata.fix.time.tv_nsec * 1e-9) * 1e6);
                    log_info("RAD[%lf] \t Time Delta: %lf", radius/1e6, (timestamp - (simulator.gpsdata.gpsdata.fix.time.tv_sec + simulator.gpsdata.gpsdata.fix.time.tv_nsec * 1e-9) * 1e6) / 1e6);
                }
                else
                {
                    log_error("Error making request: %d", craggyResult);
                    goto error;
                }
                log_info("--------------- STOP ---------------");
            }
            repeats--;
            simulator.tp_lock = false;
        }
    }

    log_warn("Terminating.... ");

    goto exit;
error:
    assert(result != 0);

exit:
    free(hostname);
    free(publicKey);
    simulator.gps_serial_thread_exit = true;
    return 0;
}
