/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Paul Wilkening, Rachel Hegeman
  Created on: 2014

 (C) Copyright 2014 Johns Hopkins University (JHU), All Rights Reserved.

 --- begin cisst license - do not edit ---

 This software is provided "as is" under an open source license, with
 no warranty.  The complete license can be found in license.txt and
 http://www.cisst.org/cisst/license.txt.

 --- end cisst license ---
 */

#ifndef _mtsVFController_h
#define _mtsVFController_h

#if sawConstraintController_USE_EIGEN
#include <sawConstraintController/constraintOptimizer.h>
#else
#include <cisstNumerical/nmrConstraintOptimizer.h>
#endif

#include <sawConstraintController/mtsVFBase.h>
#include <sawConstraintController/prmKinematicsState.h>
#include <sawConstraintController/prmSensorState.h>
#include <sawConstraintController/prmOffsetState.h>
#include <cisstParameterTypes/prmStateJoint.h>

// Always include last!
#include <sawConstraintController/sawConstraintControllerExport.h>

/*! \brief mtsVFController: A class that is responsible for managing the
    virtual fixtures, relevant state data, and the control optimizer
 */
class CISST_EXPORT mtsVFController: public cmnGenericObject
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_VERBOSE);

public:

    //Current mode of controller (what controllerOutput represents)
    //The possible values of MODE refer to:
    //1. Treating controllerOutput as an incremental joint position
    //2. Treating controllerOutput as an incremental cartesian position
    mtsVFBase::CONTROLLERMODE ControllerMode;

    /*! Constructor
    */
    mtsVFController(){}

    /*! Constructor
    */
    mtsVFController(size_t num_joints, mtsVFBase::CONTROLLERMODE cm):
        Optimizer(num_joints)
    {        
        ControllerMode = cm;
    }

    ~mtsVFController()
    {
        std::map<std::string,mtsVFBase *>::iterator itVF;
        for(itVF = VFMap.begin(); itVF != VFMap.end(); itVF++)
        {       
            delete itVF->second;
        }

        std::map<std::string,prmKinematicsState *>::iterator itKin;
        for(itKin = Kinematics.begin(); itKin != Kinematics.end(); itKin++)
        {       
            delete itKin->second;
        }

        std::map<std::string,prmSensorState *>::iterator itSens;
        for(itSens = Sensors.begin(); itSens != Sensors.end(); itSens++)
        {       
            delete itSens->second;
        }
    }

    nmrConstraintOptimizer GetOptimizer(){return Optimizer;}

    bool ActivateVF(const std::string & s);

    void DeactivateAll(); 

    //! Adds/Updates a sensor to the map
    void SetSensor(const prmSensorState & sen);

    //! Adds/Updates a sensor to the map
    void SetSensorOffset(const prmOffsetState & sen);

    //! Removes a sensor from the map
    void RemoveSensorFromMap(const std::string & senName);

    //! Finds the "base" object for kinematics and sensor data that has an offset
    void LookupBaseData(void);

    void UpdateKinematics(void);

    //! Removes a kinematics object from the map
    void RemoveKinematicsFromMap(const std::string & kinName);

    //! Updates the robot state data and control optimizer
    void UpdateOptimizer(double TickTime);

    //! Solves the constraint optimization problem and fills the result into the parameter
    nmrConstraintOptimizer::STATUS Solve(vctDoubleVec & dq);

    //map between string names and pointers to virtual fixtures
    std::map<std::string, mtsVFBase*> VFMap;

    //map between string names and pointers to kinematics objects
    std::map<std::string, prmKinematicsState*> Kinematics;

    //map between string names and pointers to sensor objects
    std::map<std::string, prmSensorState*> Sensors;

    //control optimizer variables
    nmrConstraintOptimizer Optimizer;

    //! Helper function that increments users of new vf
    void IncrementUsers(const std::vector<std::string> kin_names,
        const std::vector<std::string> sensor_names);

    //! Helper function that decrements users of new data in an old vf
    void DecrementUsers(const std::vector<std::string> kin_names,
        const std::vector<std::string> sensor_names);

    template<typename VFT, typename DT> void SetVF(const DT &vf);
    template<typename DT> bool SetVFData(const DT &data);
    template<typename KT> void SetKinematics(const KT &kin);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsVFController);

#include "mtsVFController-inl.h"

#endif // _mtsVFController_h

