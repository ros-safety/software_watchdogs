// Copyright (c) 2020 Mapless AI, Inc. Based on code that is
// Copyright 2020 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SW_WATCHDOG__VISIBILITY_CONTROL_H_
#define SW_WATCHDOG__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SW_WATCHDOG_EXPORT __attribute__ ((dllexport))
    #define SW_WATCHDOG_IMPORT __attribute__ ((dllimport))
  #else
    #define SW_WATCHDOG_EXPORT __declspec(dllexport)
    #define SW_WATCHDOG_IMPORT __declspec(dllimport)
  #endif
  #ifdef SW_WATCHDOG_BUILDING_DLL
    #define SW_WATCHDOG_PUBLIC SW_WATCHDOG_EXPORT
  #else
    #define SW_WATCHDOG_PUBLIC SW_WATCHDOG_IMPORT
  #endif
  #define SW_WATCHDOG_PUBLIC_TYPE SW_WATCHDOG_PUBLIC
  #define SW_WATCHDOG_LOCAL
#else
  #define SW_WATCHDOG_EXPORT __attribute__ ((visibility("default")))
  #define SW_WATCHDOG_IMPORT
  #if __GNUC__ >= 4
    #define SW_WATCHDOG_PUBLIC __attribute__ ((visibility("default")))
    #define SW_WATCHDOG_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SW_WATCHDOG_PUBLIC
    #define SW_WATCHDOG_LOCAL
  #endif
  #define SW_WATCHDOG_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // SW_WATCHDOG__VISIBILITY_CONTROL_H_
