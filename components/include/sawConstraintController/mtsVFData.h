#ifndef _mtsVFData_h
#define _mtsVFData_h

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <cisstVector/vctTransformationTypes.h>

struct mtsVFDataBase
{
  std::string Name;
  double Importance;
  std::vector<std::string> KinNames;
  std::vector<std::string> SensorNames;
  vctDynamicVector<double> SlackLimits;
  vctDynamicVector<double> SlackCosts;
  size_t NumSlacks;
  vctDynamicVector<size_t> DOFSelections;
  size_t ObjectiveRows;
  size_t IneqConstraintRows;
  size_t EqConstraintRows;
  vctDynamicMatrix<double> ObjectiveMatrix;
  vctDynamicVector<double> ObjectiveVector;
  vctDynamicMatrix<double> IneqConstraintMatrix;
  vctDynamicVector<double> IneqConstraintVector;
  vctDynamicMatrix<double> EqConstraintMatrix;
  vctDynamicVector<double> EqConstraintVector;

  mtsVFDataBase();
  mtsVFDataBase(const mtsVFDataBase&);
  virtual ~mtsVFDataBase();
  std::string HumanReadable(void) const;
};

struct mtsVFDataAbsoluteJointLimits : mtsVFDataBase
{
  vctDoubleVec CurrentJoints;
  vctDoubleVec LowerLimits;
  vctDoubleVec UpperLimits;

  mtsVFDataAbsoluteJointLimits();
  mtsVFDataAbsoluteJointLimits(const mtsVFDataAbsoluteJointLimits&);
  ~mtsVFDataAbsoluteJointLimits();
  std::string HumanReadable(void) const;
};

struct mtsVFDataJointLimits : mtsVFDataBase
{
  vctDoubleVec LowerLimits;
  vctDoubleVec UpperLimits;

  mtsVFDataJointLimits();
  mtsVFDataJointLimits(const mtsVFDataJointLimits&);
  ~mtsVFDataJointLimits();
  std::string HumanReadable(void) const;
};

struct mtsVFDataPlane : mtsVFDataBase
{
  vct3 Normal;

  mtsVFDataPlane();
  mtsVFDataPlane(const mtsVFDataPlane&);
  ~mtsVFDataPlane();
  std::string HumanReadable(void) const;
};

#endif
