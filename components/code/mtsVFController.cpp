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

#include <sawConstraintController/mtsVFController.h>

// CMN_IMPLEMENT_SERVICES(mtsVFController)

//! Adds/updates a sensor compliance virtual fixture in the map and increments users of kinematics and sensors
/*! SetVFSensorCompliance
@param vf virtual fixture to be added
*/
bool mtsVFController::ActivateVF(const std::string & s)
{
    // find vf by data.Name
    std::map<std::string, mtsVFBase *>::iterator itVF;
    itVF = VFMap.find(s);

    // if not found, return false
    if(itVF == VFMap.end())
    {
        return false;
    }
    itVF->second->Active = true;
    return true;    
}

void mtsVFController::DeactivateAll()
{
    std::map<std::string,mtsVFBase *>::iterator itVF;
    for(itVF = VFMap.begin(); itVF != VFMap.end(); itVF++)
    {
        if(itVF->second->Active)
        {
            itVF->second->Active = false;
        }
    }
}

void mtsVFController::UpdateKinematics()
{
    std::map<std::string, prmKinematicsState*>::iterator itKin;
    for (itKin = Kinematics.begin(); itKin != Kinematics.end(); itKin++)
    {
      itKin->second->Update();
    }
}

//! Removes a kinematics object from the map
/*! RemoveKinematicsFromMap
@param kinName name of kinematics object to be removed
*/
void mtsVFController::RemoveKinematicsFromMap(const std::string & kinName)
{
    // Removes a kinematics object from the map
    std::map<std::string, prmKinematicsState *>::iterator itKin;
    itKin = Kinematics.find(kinName);
    if(itKin == Kinematics.end())
    {
        return;  // not found, so no action required
    }

    prmKinematicsState * kinP = itKin->second;

    // Now remove from the map and delete the old object
    Kinematics.erase(itKin);
    delete kinP;
}

//! Adds/updates a sensor in the map
/*! SetSensor
@param sen sensor state to be added
*/
void mtsVFController::SetSensor(const prmSensorState & sen)
{
    RemoveSensorFromMap(sen.Name);
    Sensors.insert(std::pair<std::string, prmSensorState *>(sen.Name,new prmSensorState(sen)));
}

//! Removes a sensor from the map
/*! RemoveSensorFromMap
@param senName name of sensor state to be removed
*/
void mtsVFController::RemoveSensorFromMap(const std::string & senName)
{
    // Removes a sensor from the map
    std::map<std::string, prmSensorState *>::iterator itSen;
    itSen = Sensors.find(senName);
    if(itSen == Sensors.end())
    {
        return;  // not found, so no action required
    }

    prmSensorState * senP = itSen->second;

    // Now remove from the map and delete the old object
    Sensors.erase(itSen);
    delete senP;
}

//! Reallocates the tableau, assigns references to it for the virtual fixtures, and instructs the virtual fixtures to fill their references in
/*! UpdateCO
*/
void mtsVFController::UpdateOptimizer(double TickTime)
{
    // use VFVector to find the space needed in the control optimizer tableau
    Optimizer.ResetIndices();

    std::map<std::string,mtsVFBase*>::iterator itVF;
    for(itVF = VFMap.begin(); itVF != VFMap.end(); itVF++)
    {       
        if(itVF->second->Active)
        {
            itVF->second->ReserveSpace(Optimizer);
        }
    }


    //allocate the control optimizer matrices and vectors according to its indices
    Optimizer.Allocate();

    // go through virtual fixtures again and fill in tableau
    Optimizer.ResetIndices();    

    for(itVF = VFMap.begin(); itVF != VFMap.end(); itVF++)
    {
        mtsVFBase * tempVFData = itVF->second;        

        if(tempVFData->Active)
        {
            
            //updates the virtual fixture's kinematics and sensor objects
            tempVFData->LookupStateData(Kinematics,Sensors);

            //updates the virtual fixture's references to the control optimizer tableau
            tempVFData->SetTableauRefs(Optimizer);

            //fills in the tableau (subclasses of mtsVFData override this method with their own logic)
            tempVFData->FillInTableauRefs(ControllerMode,TickTime);

        }
    }
}

//! Solves the constraint optimization problem and fills the result into the parameter
/*! Solve
  @param dq Storage for the result of the solve call
  */
nmrConstraintOptimizer::STATUS mtsVFController::Solve(vctDoubleVec & dq)
{
    return Optimizer.Solve(dq);
}

void mtsVFController::LookupBaseData()
{
    std::map<std::string, prmKinematicsState *>::iterator kinIt;
    for(kinIt = Kinematics.begin(); kinIt != Kinematics.end(); kinIt++)
    {
        if(kinIt->second->NeedsBase)
        {
            kinIt->second->LookupKinematics(Kinematics);
        }
    }
    std::map<std::string, prmSensorState *>::iterator senIt;
    for(senIt = Sensors.begin(); senIt != Sensors.end(); senIt++)
    {
        if(senIt->second->NeedsBase)
        {
            senIt->second->LookupSensor(Sensors);
        }
    }
}

//! Helper function for incrementing the users of sensors and kinematics that a new VF requires
/*! IncrementUsers
  */
void mtsVFController::IncrementUsers(const std::vector<std::string> kin_names,
    const std::vector<std::string> sensor_names)
{
    std::map<std::string,prmKinematicsState *>::iterator itKin;
    std::map<std::string,prmSensorState *>::iterator itSen;

    //increment kinematics users
    for(size_t i = 0; i < kin_names.size(); i++)
    {
        itKin = Kinematics.find(kin_names.at(i));
        if(itKin != Kinematics.end())
        {
            itKin->second->UserCount++;
        }
    }

    //increment sensor users
    for(size_t i = 0; i < sensor_names.size(); i++)
    {
        itSen = Sensors.find(sensor_names.at(i));
        if(itSen != Sensors.end())
        {
            itSen->second->UserCount++;
        }
    }
}

void mtsVFController::DecrementUsers(const std::vector<std::string> kin_names,
    const std::vector<std::string> sensor_names)
{
    // Reduce the user counts for any kinematics objects used by this VF
    std::map<std::string,prmKinematicsState *>::iterator itKin;
    for(size_t i = 0; i < kin_names.size(); i++)
    {
        itKin = Kinematics.find(kin_names.at(i));
        if(itKin != Kinematics.end())
        {
            itKin->second->UserCount--;
        }
    }

    // Similarly, reduce the user counts for sensor objects used by this VF
    std::map<std::string,prmSensorState *>::iterator itSen;
    for(size_t i = 0; i < sensor_names.size(); i++)
    {
        itSen = Sensors.find(sensor_names.at(i));
        if(itSen != Sensors.end())
        {
            itSen->second->UserCount--;
        }
    }
}

