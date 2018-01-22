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

#ifndef _mtsVFJointLimits_h
#define _mtsVFJointLimits_h

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <sawConstraintController/mtsVFBase.h>
#include <sawConstraintController/mtsVFData.h>

/*! \brief mtsVFJointLimits: A class that contains logic for the implementation of  Plane virtual fixtures
 */
class mtsVFJointLimits : public mtsVFBase
{

public:

    /*! Constructor
    */
    mtsVFJointLimits() : mtsVFBase(){}

    /*! Constructor
    \param name String name of object
    */
    mtsVFJointLimits(const std::string & name, mtsVFDataJointLimits * data) : mtsVFBase(name,data){}

    //! Updates co with virtual fixture data.
    /*! FillInTableauRefs
    */
    void FillInTableauRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime);

};

#endif // _mtsVFJointLimits_h
