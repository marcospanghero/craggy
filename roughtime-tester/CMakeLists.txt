# Copyright 2020 Johan Lindquist
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

cmake_minimum_required(VERSION 3.10)

project(roughtime-tester C)

set(SOURCES
        others/log
        serial_api/driver_ubx
        serial_api/gpsd
        serial_api/serial-driver
        serial_api/gpsutils
        serial_api/serial
        serial_api/subframe
        base64
        main)

add_executable(roughtime-tester ${SOURCES})
target_link_libraries(roughtime-tester craggy)
target_link_libraries(roughtime-tester m)

add_compile_options(-DLOG_USE_COLOR)

find_package (Threads)
target_link_libraries (roughtime-tester ${CMAKE_THREAD_LIBS_INIT})


if (CRAGGY_WITH_OPENSSL_BINDINGS)
    find_package(OpenSSL REQUIRED)
    target_link_libraries(roughtime-tester OpenSSL::SSL)
endif()
