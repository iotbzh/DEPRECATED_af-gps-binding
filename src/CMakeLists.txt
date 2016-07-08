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

cmake_minimum_required(VERSION 3.3)

###########################################################################

link_libraries(-Wl,--as-needed -Wl,--gc-sections)

add_compile_options(-Wall -Wextra -Wconversion)
add_compile_options(-Wno-unused-parameter) # frankly not using a parameter does it care?
add_compile_options(-Werror=maybe-uninitialized)
add_compile_options(-Werror=implicit-function-declaration)
add_compile_options(-ffunction-sections -fdata-sections)
add_compile_options(-Wl,--as-needed -Wl,--gc-sections)
add_compile_options(-fPIC)

set(CMAKE_C_FLAGS_PROFILING    "-g -O0 -pg -Wp,-U_FORTIFY_SOURCE")
set(CMAKE_C_FLAGS_DEBUG        "-g -O0 -ggdb -Wp,-U_FORTIFY_SOURCE")
set(CMAKE_C_FLAGS_RELEASE      "-g -O2")
set(CMAKE_C_FLAGS_CCOV         "-g -O2 --coverage")

###########################################################################

include(FindPkgConfig)

pkg_check_modules(EXTRAS REQUIRED json-c libsystemd afb-daemon)
add_compile_options(${EXTRAS_CFLAGS})
include_directories(${EXTRAS_INCLUDE_DIRS})
link_libraries(${EXTRAS_LIBRARIES})

###########################################################################
# the binding for afb

message(STATUS "Creation af-gps-binding for AFB-DAEMON")

###############################################################
#pkg_get_variable(afb_binding_install_dir afb-daemon binding_install_dir)
execute_process(
	COMMAND pkg-config --variable binding_install_dir afb-daemon
	OUTPUT_VARIABLE afb_binding_install_dir OUTPUT_STRIP_TRAILING_WHITESPACE
)

###############################################################
add_library(af-gps-binding MODULE af-gps-binding.c)
set_target_properties(af-gps-binding PROPERTIES
	PREFIX ""
	LINK_FLAGS "-Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/export.map"
)
install(TARGETS af-gps-binding LIBRARY DESTINATION ${afb_binding_install_dir})

