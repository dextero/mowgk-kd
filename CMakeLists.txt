cmake_minimum_required(VERSION 2.8.5)
set(CMAKE_CXX_COMPILER g++)

project(kd)

file(GLOB CUSTOM_MODULES "${CMAKE_SOURCE_DIR}/cmake/*")
foreach(MODULE ${CUSTOM_MODULES})
    include(${MODULE})
endforeach()

# project properties
set(ROOT_DIR ${PROJECT_SOURCE_DIR})
set(LIBRARY_OUTPUT_PATH ${ROOT_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${ROOT_DIR}/bin)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/extlibs ${CMAKE_MODULE_PATH})
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -pg -O0 -D_DEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -DNDEBUG")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)

if(NOT DEFINED ENV{SDK_DIR})
    set(ENV{SDK_DIR} "${ROOT_DIR}/extlibs")
endif()

# setup library directories
if(DEFINED ENV{SDK_DIR})
    file(TO_CMAKE_PATH $ENV{SDK_DIR} SDK_DIR)
endif()

# external libraries
add_external_library(CGAL)
add_external_library(Sandbox)

# project sources
include_directories(${ROOT_DIR}/include)
find_sources(HEADERS ${ROOT_DIR}/include ".h" "src")
find_sources(CXX_SOURCES ${ROOT_DIR}/src ".cpp" "src")
set(SOURCES ${HEADERS}
            ${CXX_SOURCES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++14 -frounding-math")

add_executable(kd ${SOURCES})
if(LIBS)
    target_link_libraries(kd ${LIBS})
endif()
