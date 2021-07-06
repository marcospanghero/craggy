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

#include "CraggyTimeUtils.h"

#define NUMBER_OF_JULIAN_DAYS_UNTIL_EPOCH 40587
#define NUMBER_OF_MICROSECONDS_IN_SECOND 1000000
#define NUMBER_OF_SECONDS_IN_A_DAY (60 * 60 * 24) // 86400 seconds / 24 hrs

CraggyResult craggy_roughtimeToEpoc(craggy_roughtime_result *roughtimeResult, uint64_t serverRoundTripUs, uint64_t *outTime) {
/*
 5.1.5.  Timestamp

   A timestamp is a uint64 interpreted in the following way.  The most
   significant 3 bytes contain the integer part of a Modified Julian
   Date (MJD).  The least significant 5 bytes is a count of the number
   of Coordinated Universal Time (UTC) microseconds [ITU-R_TF.460-6]
   since midnight on that day.

   The MJD is the number of UTC days since 17 November 1858
   [ITU-R_TF.457-2].  It is useful to note that 1 January 1970 is 40,587
   days after 17 November 1858.

   Note that, unlike NTP, this representation does not use the full
   number of bits in the fractional part and that days with leap seconds
   will have more or fewer than the nominal 86,400,000,000 microseconds.
 */
    *outTime = ((roughtimeResult->time >> (uint64_t )40) - NUMBER_OF_JULIAN_DAYS_UNTIL_EPOCH) * NUMBER_OF_SECONDS_IN_A_DAY;
    *outTime += (((roughtimeResult->time << (uint64_t )24) >> (uint64_t )24)+(serverRoundTripUs/2))/NUMBER_OF_MICROSECONDS_IN_SECOND;

    return CraggyResultSuccess;
}