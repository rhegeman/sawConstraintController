#
# (C) Copyright 2014-2016 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

project (sawConstraintController)
cmake_minimum_required (VERSION 2.8)


# find required dependencies
set(EIGEN3_INCLUDE_DIR "Eigen include directory" CACHE PATH "")
include_directories(${EIGEN3_INCLUDE_DIR})

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstOSAbstraction
     cisstMultiTask
     cisstParameterTypes     
     cisstNumerical
     )

find_package (cisst 1.0.8 REQUIRED COMPONENTS ${REQUIRED_CISST_LIBRARIES})

# load cisst configuration
include (${CISST_USE_FILE})

set (sawConstraintController_CONFIG_FILE_DIR "${sawConstraintController_BINARY_DIR}")

# create/configure file for find_package (sawConstraintController)
set (sawConstraintController_INCLUDE_DIR
   "${sawConstraintController_SOURCE_DIR}/include"
   "${sawConstraintController_BINARY_DIR}/include")
set (sawConstraintController_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
set (sawConstraintController_LIBRARIES sawConstraintController)

include_directories (${sawConstraintController_INCLUDE_DIR})
set (sawConstraintController_HEADER_DIR "include/sawConstraintController")

set (HEADER_FILES
   ${sawConstraintController_HEADER_DIR}/mtsVFBase.h
   ${sawConstraintController_HEADER_DIR}/mtsVFJointVel.h
   ${sawConstraintController_HEADER_DIR}/mtsVFController.h
   ${sawConstraintController_HEADER_DIR}/mtsVFPlane.h
   ${sawConstraintController_HEADER_DIR}/mtsVFJointLimits.h
   ${sawConstraintController_HEADER_DIR}/mtsVFAbsoluteJointLimits.h
   ${sawConstraintController_HEADER_DIR}/mtsVF_RCM.h
   ${sawConstraintController_HEADER_DIR}/constraintOptimizer.h
   ${sawConstraintController_HEADER_DIR}/prmState.h
   ${sawConstraintController_HEADER_DIR}/mtsVFData.h
 )

  set (SOURCE_FILES
       code/mtsVFBase.cpp
       code/mtsVFJointVel.cpp
       code/mtsVFController.cpp
       code/mtsVFPlane.cpp
       code/mtsVFJointLimits.cpp
       code/mtsVFAbsoluteJointLimits.cpp
       code/mtsVF_RCM.cpp
       code/constraintOptimizer.cpp
       code/eiquadprog.hpp
       code/prmState.cpp
       code/mtsVFData.cpp
       )

add_library (sawConstraintController
               ${HEADER_FILES}
               ${SOURCE_FILES}
               ${sawConstraintController_CISST_DG_SRCS}
               ${sawConstraintController_CISST_DG_HDRS})

set_property (TARGET sawConstraintController PROPERTY FOLDER "sawConstraintController")
cisst_target_link_libraries (sawConstraintController ${REQUIRED_CISST_LIBRARIES})

set (sawConstraintController_CMAKE_CONFIG_FILE "${sawConstraintController_CONFIG_FILE_DIR}/sawConstraintControllerConfig.cmake")
configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/cmake/sawConstraintControllerConfig.cmake.in"
                "${sawConstraintController_CMAKE_CONFIG_FILE}"
                @ONLY)

cisst_add_config_version ("${sawConstraintController_CONFIG_FILE_DIR}/sawConstraintControllerConfigVersion.cmake"
                            VERSION ${sawConstraintController_VERSION}
                            DESTINATION "share/sawConstraintController"
                            COMPONENT sawConstraintController)

# Install targets for headers and library
install (DIRECTORY
         "${sawConstraintController_SOURCE_DIR}/include/sawConstraintController"
         "${sawConstraintController_BINARY_DIR}/include/sawConstraintController"
         DESTINATION include
         PATTERN .svn EXCLUDE)

install (TARGETS sawConstraintController
         RUNTIME DESTINATION bin
         LIBRARY DESTINATION lib
         ARCHIVE DESTINATION lib)

install (FILES "${sawConstraintController_CMAKE_CONFIG_FILE}"
         DESTINATION "share/sawConstraintController")

