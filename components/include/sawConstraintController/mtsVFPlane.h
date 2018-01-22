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

#ifndef _mtsVFPlane_h
#define _mtsVFPlane_h

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <sawConstraintController/mtsVFBase.h>
#include <sawConstraintController/mtsVFData.h>

/*! \brief mtsVFPlane: A class that contains logic for the implementation of  Plane virtual fixtures
 */
class mtsVFPlane : public mtsVFBase
{

public:

    /*! Constructor
    */
    mtsVFPlane() : mtsVFBase(){}

    /*! Constructor
    \param name String name of object
    */
    mtsVFPlane(const std::string & name, mtsVFDataBase * data) : mtsVFBase(name,data)
    {
        IsFrameSet = false;
    }

    //! Updates co with virtual fixture data.
    /*! FillInTableauRefs
    */
    void FillInTableauRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime);

    void SetFrame(const vctFrame4x4<double>& Frame);

    prmKinematicsState * CurrentKinematics;

private:
    bool IsFrameSet;
    vctFrame4x4<double> frame;
};

#endif // _mtsVFPlane_h
