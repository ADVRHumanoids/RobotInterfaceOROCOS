# 
#  Copyright (C) 2017 IIT-ADVR
#  Author: Enrico Mingo Hoffman
#  email: enrico.mingo@iit.it
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program. If not, see <http://www.gnu.org/licenses/>
#

cmake_minimum_required(VERSION 2.8.12)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0005 NEW)
    cmake_policy(SET CMP0017 NEW)
endif(COMMAND cmake_policy)

include(ExternalProject)

project(RobotInterfaceOROCOS)


# C++ 11
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(XBotInterface REQUIRED)

find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

find_package(orocos-ocl REQUIRED)
find_package(orocos_kdl REQUIRED)


find_package(RST-RT REQUIRED)
message(STATUS "RST-RT version: ${RST-RT_VERSION}")
include_directories(BEFORE SYSTEM ${RST-RT_INCLUDE_DIRS})
add_definitions(${RST-RT_CFLAGS})
link_directories(${RST-RT_LIBRARY_DIR} ${RST-RT_LIBRARY_DIRS})

include_directories(${RTT-CORE-EXTENSIONS_INCLUDE_DIRS})
link_directories(${RTT-CORE-EXTENSIONS_LIBRARY_DIRS})
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

 MESSAGE("sssss \"${USE_OROCOS_INCLUDE_DIRS}\"")

  MESSAGE("ssssssssss \"${RTT-CORE-EXTENSIONS_INCLUDE_DIRS}\"")
  
include_directories(include 
                    ${XBotInterface_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    ${USE_OROCOS_INCLUDE_DIRS}
                    ${RST-RT_INCLUDE_DIRS})
                        
file(GLOB_RECURSE XBotInterfaceOROCOS_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include" *.h*)


add_library(RobotInterfaceOROCOS SHARED ${XBotInterfaceOROCOS_INCLUDES} 
                                        src/RobotInterfaceOROCOS.cpp)

                   
target_link_libraries(${PROJECT_NAME}
   ${USE_OROCOS_LIBRARIES}
   ${OROCOS-RTT_LIBRARIES}
   ${Boost_LIBRARIES}
   ${RST-RT_LIBRARIES}
   ${XBotInterface_LIBRARIES}
)                 

target_include_directories(RobotInterfaceOROCOS PUBLIC ${XBotInterface_INCLUDE_DIRS}
                        ${EIGEN3_INCLUDE_DIRS})


                                        
    ########################################################################

    set(VARS_PREFIX "RobotInterfaceOROCOS")

    set(RobotInterfaceOROCOS_MAJOR_VERSION 0)
    set(RobotInterfaceOROCOS_MINOR_VERSION 1)
    set(RobotInterfaceOROCOS_PATCH_VERSION 0)
    set(RobotInterfaceOROCOS_VERSION ${RobotInterfaceOROCOS_MAJOR_VERSION}.${RobotInterfaceOROCOS_MINOR_VERSION}.${RobotInterfaceOROCOS_PATCH_VERSION})

    find_package(YCM REQUIRED)
    include(YCMDefaultDirs)
    ycm_default_dirs(${VARS_PREFIX})

    target_include_directories(RobotInterfaceOROCOS PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                                    "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

    set_target_properties(RobotInterfaceOROCOS PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                                SOVERSION ${${VARS_PREFIX}_VERSION})

    install(DIRECTORY include/
            DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
            FILES_MATCHING PATTERN "*.h*")

    install(TARGETS RobotInterfaceOROCOS
            EXPORT RobotInterfaceOROCOS
            ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
            RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
            LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)



    #enabling it will add all RobotInterfaceOROCOS dependencies as dependencies for third party users
    set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS RobotInterfaceOROCOS)


    include(InstallBasicPackageFiles)
    install_basic_package_files(RobotInterfaceOROCOS VARS_PREFIX ${VARS_PREFIX}
                                        VERSION ${${VARS_PREFIX}_VERSION}
                                        COMPATIBILITY SameMajorVersion
                                        TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                        NO_CHECK_REQUIRED_COMPONENTS_MACRO)

    include(AddUninstallTarget)
####

orocos_component(robot_interface_orocos_test
tests/testRobotInterfaceOROCOS.cpp
${testRobotInterfaceOROCOS_INCLUDES})
set_property(TARGET robot_interface_orocos_test APPEND PROPERTY COMPILE_DEFINITIONS RTT_COMPONENT)

target_link_libraries(robot_interface_orocos_test
   ${OROCOS-RTT_LIBRARIES}
   ${Boost_LIBRARIES}
   ${RST-RT_LIBRARIES}
   ${XBotInterface_LIBRARIES}
   RobotInterfaceOROCOS
)

install (TARGETS
         LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
orocos_generate_package(INCLUDE_DIRS include)
