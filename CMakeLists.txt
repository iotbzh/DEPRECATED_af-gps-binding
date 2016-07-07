###########################################################################
# Copyright 2016 IoT.bzh
#
# author: José Bollo <jose.bollo@iot.bzh>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###########################################################################

project(af-gps-binding C)

cmake_minimum_required(VERSION 3.4)

include(GNUInstallDirs)

set(PROJECT_NAME "af-gps-binding")
set(PROJECT_PRETTY_NAME "af binding for GPS")
set(PROJECT_DESCRIPTION "Binding for handling NMEA signalisation of GPS")
set(PROJECT_VERSION "1.0")

set(CMAKE_BUILD_TYPE Debug)

add_subdirectory(src)


