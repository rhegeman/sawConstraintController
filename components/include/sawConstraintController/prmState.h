#ifndef _prmState_h
#define _prmState_h

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <cisstVector/vctTransformationTypes.h>

struct prmJointState
{
  vctDoubleVec Position;
  vctDoubleVec Velocity;
  vctDoubleVec Acceleration;

  prmJointState(void);
  prmJointState(const prmJointState& other);
  ~prmJointState();
};

struct prmKinematicsState
{
  vctFrm3 Frame;
  vctDoubleMat Jacobian;
  int UserCount;
  std::string Name;
  prmJointState *JointState;

  ~prmKinematicsState();

  virtual void LookupKinematics(const std::map<std::string,prmKinematicsState*>&) {}
  prmKinematicsState(void);
  prmKinematicsState(const prmKinematicsState& other);
  prmKinematicsState(const std::string &n, prmJointState *js);
  virtual void Update() {}
  std::string HumanReadable(void) const;
};

struct prmSensorState
{
  std::string Name;
  vctDynamicVector<double> Values;
  int UserCount;

  prmSensorState(void);
  prmSensorState(const std::string &n);
  prmSensorState(const prmSensorState &other);
  ~prmSensorState(void);
  virtual void LookupSensor(const std::map<std::string, prmSensorState*> &) {};

  std::string HumanReadable(void) const;

};

#endif

