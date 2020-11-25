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

#ifndef CRAGGY_CRAGGYTIMEUTILS_H
#define CRAGGY_CRAGGYTIMEUTILS_H

#include "CraggyTypes.h"


CraggyResult craggy_roughtimeToEpoc(craggy_roughtime_result *roughtimeResult, uint64_t *outTime);

#endif //CRAGGY_CRAGGYTIMEUTILS_H
