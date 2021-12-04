#
#   Copyright 2016 rm5248
#   updated 2021 alexb@clever.team
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
PROJECT(cserial_example)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

IF( UNIX )
    find_package(PkgConfig)
    pkg_search_module( CSERIAL REQUIRED cserial )
    find_package(Threads REQUIRED)
ENDIF( UNIX )

IF( WIN32 )
    SET( C_SERIAL_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/../include )
    INCLUDE_DIRECTORIES( ${C_SERIAL_INCLUDE_DIRECTORIES} ${CMAKE_CURRENT_BINARY_DIR} )
ENDIF( WIN32 )

add_executable( cserial-example example_full.c )
target_link_libraries( cserial-example ${CSERIAL_LIBRARIES} )

add_executable( simpleterm serial_simple_term.cpp )
target_link_libraries( simpleterm ${CSERIAL_LIBRARIES} )

if(THREADS_HAVE_PTHREAD_ARG)
  target_compile_options(cserial-example PUBLIC "-pthread")
  target_compile_options(simpleterm PUBLIC "-pthread")
endif()
if(CMAKE_THREAD_LIBS_INIT)
  target_link_libraries(cserial-example "${CMAKE_THREAD_LIBS_INIT}")
  target_link_libraries(simpleterm "${CMAKE_THREAD_LIBS_INIT}")
endif()
