#include <sawConstraintController/mtsVFData.h>

mtsVFDataBase::mtsVFDataBase()
    : Name()
    , Importance(1.0)
    , KinNames()
    , SensorNames()
    , SlackLimits()
    , SlackCosts()
    , NumSlacks(0)
    , DOFSelections()
    , ObjectiveRows(0)
    , IneqConstraintRows(0)
    , EqConstraintRows(0)
    , ObjectiveMatrix()
    , ObjectiveVector()
    , IneqConstraintMatrix()
    , IneqConstraintVector()
    , EqConstraintMatrix()
    , EqConstraintVector()
{
  std::cout << "NumSlacks = " << NumSlacks << std::endl;
}

mtsVFDataBase::mtsVFDataBase(const mtsVFDataBase &other)
    : Name(other.Name)
    , Importance(other.Importance)
    , KinNames(other.KinNames)
    , SensorNames(other.SensorNames)
    , SlackLimits(other.SlackLimits)
    , SlackCosts(other.SlackCosts)
    , NumSlacks(other.NumSlacks)
    , DOFSelections(other.DOFSelections)
    , ObjectiveRows(other.ObjectiveRows)
    , IneqConstraintRows(other.IneqConstraintRows)
    , EqConstraintRows(other.EqConstraintRows)
    , ObjectiveMatrix(other.ObjectiveMatrix)
    , ObjectiveVector(other.ObjectiveVector)
    , IneqConstraintMatrix(other.IneqConstraintMatrix)
    , IneqConstraintVector(other.IneqConstraintVector)
    , EqConstraintMatrix(other.EqConstraintMatrix)
    , EqConstraintVector(other.EqConstraintVector)
{
}

mtsVFDataBase::~mtsVFDataBase()
{
}

std::string mtsVFDataBase::HumanReadable(void) const
{
    std::stringstream description__cdg;
    description__cdg << "mtsVFDataBase" << std::endl;
/*
    description__cdg << "  Name:" << cmnData<std::string >::HumanReadable(this->Name);
    description__cdg << "  Importance:" << cmnData<double >::HumanReadable(this->Importance);
    description__cdg << "  KinNames:" << cmnData<std::vector<std::string> >::HumanReadable(this->KinNames);
    description__cdg << "  SensorNames:" << cmnData<std::vector<std::string> >::HumanReadable(this->SensorNames);
    description__cdg << "  SlackLimits:" << cmnData<vctDynamicVector<double> >::HumanReadable(this->SlackLimits);
    description__cdg << "  SlackCosts:" << cmnData<vctDynamicVector<double> >::HumanReadable(this->SlackCosts);
    description__cdg << "  NumSlacks:" << cmnData<size_t >::HumanReadable(this->NumSlacks);
    description__cdg << "  DOFSelections:" << cmnData<vctDynamicVector<size_t> >::HumanReadable(this->DOFSelections);
    description__cdg << "  ObjectiveRows:" << cmnData<size_t >::HumanReadable(this->ObjectiveRows);
    description__cdg << "  IneqConstraintRows:" << cmnData<size_t >::HumanReadable(this->IneqConstraintRows);
    description__cdg << "  EqConstraintRows:" << cmnData<size_t >::HumanReadable(this->EqConstraintRows);
    description__cdg << "  ObjectiveMatrix:" << cmnData<vctDynamicMatrix<double> >::HumanReadable(this->ObjectiveMatrix);
    description__cdg << "  ObjectiveVector:" << cmnData<vctDynamicVector<double> >::HumanReadable(this->ObjectiveVector);
    description__cdg << "  IneqConstraintMatrix:" << cmnData<vctDynamicMatrix<double> >::HumanReadable(this->IneqConstraintMatrix);
    description__cdg << "  IneqConstraintVector:" << cmnData<vctDynamicVector<double> >::HumanReadable(this->IneqConstraintVector);
    description__cdg << "  EqConstraintMatrix:" << cmnData<vctDynamicMatrix<double> >::HumanReadable(this->EqConstraintMatrix);
    description__cdg << "  EqConstraintVector:" << cmnData<vctDynamicVector<double> >::HumanReadable(this->EqConstraintVector);
*/
    return description__cdg.str();
}

mtsVFDataPlane::mtsVFDataPlane(void):
      mtsVFDataBase()
    , Normal(vct3(0.0,0.0,1.0))
{}

mtsVFDataPlane::mtsVFDataPlane(const mtsVFDataPlane & other):
      mtsVFDataBase(other)
    , Normal(other.Normal)
{}

mtsVFDataPlane::~mtsVFDataPlane(void){}

std::string mtsVFDataPlane::HumanReadable(void) const {
    std::stringstream description__cdg;
    description__cdg << "mtsVFDataPlane" << std::endl;
    // description__cdg << cmnData< mtsVFDataBase >::HumanReadable(*this) << std::endl;
    // description__cdg << "  Normal:" << mtsVFDataBase::HumanReadable(this->Normal);
    return description__cdg.str();
}

mtsVFDataAbsoluteJointLimits::mtsVFDataAbsoluteJointLimits(void):
      mtsVFDataBase()
    , CurrentJoints()
    , LowerLimits()
    , UpperLimits()
{}

mtsVFDataAbsoluteJointLimits::mtsVFDataAbsoluteJointLimits(const mtsVFDataAbsoluteJointLimits & other):
      mtsVFDataBase(other)
    , CurrentJoints(other.CurrentJoints)
    , LowerLimits(other.LowerLimits)
    , UpperLimits(other.UpperLimits)
{}

mtsVFDataAbsoluteJointLimits::~mtsVFDataAbsoluteJointLimits(void){}

std::string mtsVFDataAbsoluteJointLimits::HumanReadable(void) const {
    std::stringstream description__cdg;
    description__cdg << "mtsVFDataAbsoluteJointLimits" << std::endl;
    // description__cdg << mtsVFDataBase::HumanReadable(*this) << std::endl;
    // description__cdg << "  CurrentJoints:" << cmnData<vctDoubleVec >::HumanReadable(this->CurrentJoints);
    // description__cdg << "  LowerLimits:" << cmnData<vctDoubleVec >::HumanReadable(this->LowerLimits);
    // description__cdg << "  UpperLimits:" << cmnData<vctDoubleVec >::HumanReadable(this->UpperLimits);
    return description__cdg.str();
}

mtsVFDataJointLimits::mtsVFDataJointLimits(void):
      mtsVFDataBase()
    , LowerLimits()
    , UpperLimits()
{}

mtsVFDataJointLimits::mtsVFDataJointLimits(const mtsVFDataJointLimits & other):
      mtsVFDataBase(other)
    , LowerLimits(other.LowerLimits)
    , UpperLimits(other.UpperLimits)
{}

mtsVFDataJointLimits::~mtsVFDataJointLimits(void){}

std::string mtsVFDataJointLimits::HumanReadable(void) const {
    std::stringstream description__cdg;
    description__cdg << "mtsVFDataJointLimits" << std::endl;
    // description__cdg << mtsVFDataBase::HumanReadable(*this) << std::endl;
    //description__cdg << "  LowerLimits:" << cmnData<vctDoubleVec >::HumanReadable(this->LowerLimits);
    //description__cdg << "  UpperLimits:" << cmnData<vctDoubleVec >::HumanReadable(this->UpperLimits);
    return description__cdg.str();
}

