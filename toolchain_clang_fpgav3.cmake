##################################################################
#
# Toolchain file for FPGA V3 (32-bit ARM architecture)
#
# Usage: cmake -DCMAKE_TOOLCHAIN_FILE=<path-to-toolchain>/toolchain_clang_fpgav3.cmake
#              [-DCLANG_VERSION=<ver>]  <path-to-source>
#
# Can specify clang version using optional -DCLANG_VERSION parameter. For example,
# use 10 for clang-10, or 15 for clang-15
#
# For clang, make sure that clang, llvm and lld are installed:
#   Linux:  sudo apt install clang llvm lld
#
# On Linux, can also use llvm.sh script to install version <ver>:
#   wget https://apt.llvm.org/llvm.sh
#   chmod a+x llvm.sh
#   sudo ./llvm.sh <ver>
#
##################################################################

# Sysroot
set (Sysroot /Users/peter/Desktop/Research/mechatronics-embedded/build/sysroot_external-prefix/src/sysroot_external)

# Determine gcc version by examining sysroot. The gcc version
# (e.g., 12.2.0) should be the only subdirectory in /usr/include/c++
set (gccIncDir "${Sysroot}/usr/include/c++")
file (GLOB gccVer RELATIVE ${gccIncDir} "${gccIncDir}/*")
list (LENGTH gccVer gccVerLen)
if (gccVerLen EQUAL 1)
  message (STATUS "Found gcc ${gccVer} in sysroot")
else ()
  message (SEND_ERROR "Could not determine gcc version from ${gccIncDir}")
endif ()

list (APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES gccVer)

# Set Arch to "arm32" because it is used by mechatronics-software CMake files
set (Arch "arm32" CACHE STRING "ARM arch: arm64 or arm32")

set (CMAKE_SYSTEM_NAME Linux)
set (CMAKE_SYSTEM_PROCESSOR arm)

set (gnuArch       arm-linux-gnueabihf)
set (sysrootPath   arm-xilinx-linux-gnueabi)
set (sysrootPathHf arm-xilinx-linux-gnueabihf)

##################################################################
# Find clang and set up CMake compiler definitions

if (CLANG_VERSION)
  set (CLANG_SUFFIX "-${CLANG_VERSION}")
else ()
  set (CLANG_SUFFIX "")
endif ()

find_program (CLANG_CC "clang${CLANG_SUFFIX}" REQUIRED)

set (CMAKE_C_COMPILER          "clang${CLANG_SUFFIX}")
set (CMAKE_C_COMPILER_TARGET   ${gnuArch})
set (CMAKE_CXX_COMPILER        "clang++${CLANG_SUFFIX}")
set (CMAKE_CXX_COMPILER_TARGET ${gnuArch})
set (CMAKE_ASM_COMPILER        "clang${CLANG_SUFFIX}")
set (CMAKE_STRIP               "llvm-strip${CLANG_SUFFIX}")
set (CLANG_LINKER              "lld${CLANG_SUFFIX}")

##################################################################
# Set up cross compilation paths
set (CMAKE_SYSROOT ${Sysroot})
set (CMAKE_FIND_ROOT_PATH ${Sysroot})
set (CMAKE_SKIP_BUILD_RPATH FALSE)
set (CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

# Ensure that we build relocatable binaries
set (CMAKE_INSTALL_RPATH $ORIGIN)
set (CMAKE_LIBRARY_PATH ${Sysroot}/usr/lib)
set (CMAKE_INCLUDE_PATH ${Sysroot}/usr/)

##################################################################
# 
# Search for headers and libraries in the target environment
set (CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set (CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set (CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Search for programs in the host environment
set (CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

##################################################################
# Extra compiler and linker definitions

set (CMAKE_POSITION_INDEPENDENT_CODE TRUE)

include_directories ("${Sysroot}/usr/include/c++/${gccVer}/${sysrootPath}"
                     "${Sysroot}/usr/include/c++/${gccVer}")

# Following compiler definition required for floating point hardware
add_definitions(-D__ARM_PCS_VFP)

set (EXTRA_COMPILER_FLAGS "-mfloat-abi=hard --target=${gnuArch}")

set (EXTRA_LINKER_FLAGS
     "-Wl,-z,notext -fuse-ld=${CLANG_LINKER} -B${Sysroot}/usr/lib/${sysrootPathHf}/${gccVer} -L${Sysroot}/usr/lib/${sysrootPathHf}/${gccVer}")

##################################################################
# Set CMake compiler and linker flags
#
# We expect that these flags are empty when starting CMake so do not use the usual approach
# e.g., set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EXTRA_COMPILE_FLAGS}" ...),
# because this code seems to be run twice and we end up with duplication.

set (CMAKE_C_FLAGS   "${EXTRA_COMPILER_FLAGS}" CACHE STRING "" FORCE)
set (CMAKE_CXX_FLAGS "${EXTRA_COMPILER_FLAGS}" CACHE STRING "" FORCE)
set (CMAKE_ASM_FLAGS "${EXTRA_COMPILER_FLAGS}" CACHE STRING "" FORCE)

set (CMAKE_SHARED_LINKER_FLAGS "${EXTRA_LINKER_FLAGS}" CACHE STRING "" FORCE)
# Add -pie for position-independent executable
set (CMAKE_EXE_LINKER_FLAGS    "${EXTRA_LINKER_FLAGS} -pie" CACHE STRING "" FORCE)
