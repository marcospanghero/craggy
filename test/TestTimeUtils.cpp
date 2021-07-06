/* Copyright 2020 Craggy Contributors
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

#include <cstring>
#include "gtest/gtest.h"

#include "CraggyTimeUtils.h"

namespace {

TEST(CraggyTimeUtilsTests, TestRoughtimeToEpoc) {

    // CraggyResult craggy_roughtimeToEpoc(craggy_roughtime_result *roughtimeResult, uint64_t serverRoundTripUs, uint64_t *outTime);

    craggy_roughtime_result result = {0};
    result.time=65312145749359830;
    result.radius=10000;

    uint64_t outTime = 0;
    CraggyResult craggyResult = craggy_roughtimeToEpoc(&result, 1000000, &outTime);

    EXPECT_EQ(CraggyResultSuccess,craggyResult);
    EXPECT_EQ(1625585148,outTime);

//
//    auto *originalBytes = new uint8_t[32];
//    memset(originalBytes,0,32);
//
//    auto *bytes = new uint8_t[32];
//    memset(bytes,0,32);
//
//    CraggyResult craggyResult = CraggyResultSuccess;
//
//    bool result = craggy_fillRandomBytes(bytes, (size_t)32, &craggyResult);
//
//    EXPECT_EQ(true, result);
//    EXPECT_NE(0, std::memcmp(originalBytes, bytes, 32));
//
//    delete[] bytes;

}

}  // namespace