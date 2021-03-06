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

cmake_minimum_required (VERSION 2.8)

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstOSAbstraction
     cisstNumerical
     cisstMultiTask
     cisstParameterTypes)

find_package (cisst REQUIRED)

if (cisst_FOUND)
  # Modify CMake configuration to use cisst
  include (${CISST_USE_FILE})

    find_package (sawConstraintController REQUIRED)

    if (sawConstraintController_FOUND)
        include_directories (${sawConstraintControllerExampleSimpleCommunication_SOURCE_DIR}
                             ${sawConstraintController_INCLUDE_DIR})
        link_directories (${sawConstraintController_LIBRARY_DIR})

        add_executable (sawConstraintControllerExampleSimpleCommunication
                        main.cpp
                        mtsSimpleVFController.h
                        mtsSimpleVFController.cpp
                        mtsRobotTask.cpp
                        mtsRobotTask.h
                        mtsVFControllerTask.cpp
                        mtsVFControllerTask.h
                        mtsVFSenderTask.cpp
                        mtsVFSenderTask.h
                        prmSimpleRobotKinematicsState.cpp
                        prmSimpleRobotKinematicsState.h
                        )

        target_link_libraries (sawConstraintControllerExampleSimpleCommunication
                               ${sawConstraintController_LIBRARIES})

        cisst_target_link_libraries (sawConstraintControllerExampleSimpleCommunication ${REQUIRED_CISST_LIBRARIES})

        set_property (TARGET sawConstraintControllerExampleSimpleCommunication PROPERTY FOLDER "sawConstraintController")

    endif (sawConstraintController_FOUND)

endif (cisst_FOUND)
