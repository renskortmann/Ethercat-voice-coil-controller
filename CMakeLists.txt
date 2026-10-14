cmake_minimum_required(VERSION 3.28)

project(ethercat-voice-coil-controller
   VERSION 0.1.0
   DESCRIPTION "EtherCAT master: sub-millisecond CST current-control loop for an AMC voice-coil drive"
   LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)   # main.h relies on _GNU_SOURCE

# Default to an optimized build; the RT loop wants -O2.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
   set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
   set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
      Debug Release RelWithDebInfo MinSizeRel)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- SOEM 2 -----------------------------------------------------------------
# Prefer the installed CMake package; fall back to a plain header/library search.
find_package(soem CONFIG QUIET)
if(NOT TARGET soem)
   find_path(SOEM_INCLUDE_DIR soem/soem.h)
   find_library(SOEM_LIBRARY NAMES soem)
   if(NOT SOEM_INCLUDE_DIR OR NOT SOEM_LIBRARY)
      message(FATAL_ERROR
         "SOEM 2 not found. Install it from "
         "https://github.com/OpenEtherCATsociety/SOEM (headers under 'soem/', "
         "a linkable 'soem' library), or pass -DCMAKE_PREFIX_PATH=<install-dir>.")
   endif()
   add_library(soem STATIC IMPORTED)
   set_target_properties(soem PROPERTIES
      IMPORTED_LOCATION "${SOEM_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${SOEM_INCLUDE_DIR}")
endif()

find_package(Threads REQUIRED)

# --- Executable -----------------------------------------------------------------
add_executable(ethercat-voice-coil-controller
   main.c
   fieldbus.c
   amc_config.c
   cia402.c
   control_loop.c
   open_loop_current.c
   closed_loop_position.c
   controller_pi.c
   controller_smc.c
   notch.c
   vca_ekf.c
   diagnostics.c
   logging.c)

target_link_libraries(ethercat-voice-coil-controller PRIVATE
   soem
   Threads::Threads
   m)

# Drop the built binary in the project root rather than under build/.
set_target_properties(ethercat-voice-coil-controller PROPERTIES
   RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}")

install(TARGETS ethercat-voice-coil-controller DESTINATION bin)
