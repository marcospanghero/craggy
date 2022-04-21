/* Copyright 2020 Johan Lindquist
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <string.h>
#include <time.h>
#include <stdio.h>
#include <inttypes.h>
#include <getopt.h>
#include <assert.h>

#include <unistd.h>

#include "base64.h"
#include "CraggyTransport.h"
#include "CraggyClient.h"

// TimeUs returns the current value of the specified clock in microseconds.
static uint64_t TimeUs(clockid_t clock)
{
    struct timespec tv;
    if (clock_gettime(clock, &tv))
    {
        abort();
    }
    uint64_t ret = tv.tv_sec;
    ret *= 1000000;
    ret += tv.tv_nsec / 1000;
    return ret;
}

// MonotonicUs returns the value of the monotonic clock in microseconds.
uint64_t MonotonicUs() { return TimeUs(CLOCK_MONOTONIC); }

// MonotonicUs returns the value of the realtime clock in microseconds.
uint64_t RealtimeUs() { return TimeUs(CLOCK_REALTIME); }

int main(int argc, char *argv[])
{

    CraggyResult craggyResult;

    int result = 1;

    static struct option long_options[] = {
        {"host", required_argument, 0, 'h'},
        {"key", required_argument, 0, 'k'},
        {"nonce", optional_argument, 0, 'n'},
        {"intervals", optional_argument, 0, 'i'},
        {"repreats", optional_argument, 0, 'r'},
        {0, 0, 0, 0}};

    int c;

    char *hostname = NULL;
    char *nonce = NULL;
    char *publicKey = NULL;
    uint8_t repeats = 1;
    uint8_t intervals = 1;

    while (1)
    {

        int option_index = 0;
        c = getopt_long(argc, argv, "h:k:n:i:r:", long_options, &option_index);

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

        case 'i':
            intervals = atoi(optarg);
            printf("Will poll every %d seconds\n",intervals);
            break;

        case 'r':
            repeats = atoi(optarg);
            printf("Will poll for %d times\n",repeats);
            break;

        case '?':
            /* getopt_long already printed an error message. */
            break;

        default:
            abort();
        }
    }

    if (publicKey == NULL || hostname == NULL || repeats < 1)
    {
        printf("usage: craggy -h <hostname:port> -k <public key> (-n <nonce>) - at least one request has to be sent (default is 1)");
        return 1;
    }

    if (repeats == 1) intervals = 0;
    
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

        if (craggy_createRequest(nonceBytes, requestBuf))
        {

            printf("--------------- START ---------------\n");
            const uint64_t start_us = MonotonicUs();

            craggy_rough_time_t timestamp;
            uint32_t radius;

            size_t responseBufLen = CRAGGY_ROUGH_TIME_MIN_REQUEST_SIZE * 3;
            craggy_rough_time_response_t responseBuf[responseBufLen];

            if (craggy_makeRequest(hostname, requestBuf, &craggyResult, responseBuf, &responseBufLen))
            {

                if (!craggy_processResponse(nonceBytes, rootPublicKey, responseBuf, responseBufLen, &craggyResult, &timestamp, &radius))
                {
                    printf("Error parsing response: %d", craggyResult);
                    goto error;
                }

                const uint64_t end_us = MonotonicUs();
                const uint64_t end_realtime_us = RealtimeUs();

                // We assume that the path to the Roughtime server is symmetric and thus add
                // half the round-trip time to the server's timestamp to produce our estimate
                // of the current time.
                timestamp += (end_us - start_us) / 2;
                printf("Received reply in %" PRIu64 "μs.\n", end_us - start_us);
                printf("Current time is %" PRIu64 "μs from the epoch, ±%uμs \n", timestamp, radius);
                int64_t system_offset = timestamp - end_realtime_us;
                printf("System clock differs from that estimate by %" PRId64 "μs.\n", system_offset);
                static const int64_t kTenMinutes = 10 * 60 * 1000000;

                if (imaxabs(system_offset) > kTenMinutes)
                {
                    goto error;
                }
            }
            else
            {
                printf("Error making request: %d", craggyResult);
                goto error;
            }
            printf("--------------- STOP ---------------\n");
        }

        sleep(intervals);
        repeats = repeats - 1;
    }

    goto exit;
error:
    assert(result != 0);

exit:
    free(hostname);
    free(publicKey);

    return result;
}