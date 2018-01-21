#include <sawConstraintController/mtsVF_RCM.h>

CMN_IMPLEMENT_SERVICES(mtsVF_RCM);

void mtsVF_RCM::FillInTableauRefs(const CONTROLLERMODE CMN_UNUSED(mode), const double TickTime)
{

    // Constraint-Based RCM
    vct3 Axis = -Kinematics.at(0)->Frame.Rotation() * vct3(0,0,1);
    FillMoveConstraints3D( H, h, ur, Axis,
      Kinematics.at(1)->Frame.Translation() - vct3(Sensors.at(0)->Values),
      0.002, Data->IneqConstraintRows);
    vctDoubleMat RCM_Mat = H*Kinematics.at(1)->Jacobian;

    //populate matrix and vector
    IneqConstraintMatrixRef.Assign(RCM_Mat);
    IneqConstraintVectorRef.Assign(h);

}

