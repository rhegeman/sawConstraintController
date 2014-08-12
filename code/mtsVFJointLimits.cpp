/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Preetham Chalasani
  Created on: 2014

  (C) Copyright 2014 Johns Hopkins University (JHU), All Rights Reserved.

 --- begin cisst license - do not edit ---

 This software is provided "as is" under an open source license, with
 no warranty.  The complete license can be found in license.txt and
 http://www.cisst.org/cisst/license.txt.

 --- end cisst license ---
 */

#include <sawConstraintController/mtsVFJointLimits.h>

CMN_IMPLEMENT_SERVICES(mtsVFJointLimits)

//! Updates co with virtual fixture data.
/*! FillInTableauRefs
*/
void mtsVFJointLimits::FillInTableauRefs(const CONTROLLERMODE mode, const double TickTime)
{    
    /*
         Fill in refs
         I*dq >= L
         -I*dq >= -U
    */

    if(Kinematics.size() < 1)
    {
        CMN_LOG_CLASS_RUN_ERROR << "FillInTableauRefs: Joint Limit VF given improper input" << std::endl;
        cmnThrow("FillInTableauRefs: Joint Limit VF given improper input");
    }

    mtsVFDataJointLimits * limitData = (mtsVFDataJointLimits*)(Data);

    size_t numJoints = limitData->LowerLimits.size();

    for(size_t i = 0; i < numJoints; i++)
    {
        IneqConstraintMatrixRef.SetAll(0.0);
        IneqConstraintMatrixRef.at(i,i) = 1.0;
        IneqConstraintMatrixRef.at(i+numJoints,i+numJoints) = -1.0;
        IneqConstraintVectorRef.at(i) = limitData->LowerLimits.at(i);
        IneqConstraintVectorRef.at(i+numJoints) = limitData->UpperLimits.at(i);
    }

//    std::cout << "Mat Ine \n" << IneqConstraintMatrixRef << std::endl;
//    std::cout << "Vec Ine \n" << IneqConstraintVectorRef << std::endl;
    ConvertRefs(mode,TickTime);
}