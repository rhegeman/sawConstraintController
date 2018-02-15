
#include <sawConstraintController/prmState.h>

prmKinematicsState::prmKinematicsState(void):
      Frame()
    , Jacobian()
    , UserCount()
    , Name()
    , JointState(0)
{}

prmKinematicsState::prmKinematicsState(const prmKinematicsState & other):
      Frame(other.Frame)
    , Jacobian(other.Jacobian)
    , UserCount(other.UserCount)
    , Name(other.Name)
    , JointState(other.JointState)
{}

prmKinematicsState::~prmKinematicsState(void)
{
}

prmKinematicsState::prmKinematicsState(const std::string & n, prmJointState * js)
  : Frame(),
    Jacobian()
{
    Name = n;
    UserCount = 0;
    JointState = js;
}

std::string prmKinematicsState::HumanReadable(void) const {
    std::stringstream description__cdg;
    description__cdg << "prmKinematicsState" << std::endl;
//    description__cdg << "  Frame:" << cmnData<vctFrm3 >::HumanReadable(this->Frame);
//    description__cdg << "  Jacobian:" << cmnData<vctDoubleMat >::HumanReadable(this->Jacobian);
//    description__cdg << "  UserCount:" << cmnData<int >::HumanReadable(this->UserCount);
//    description__cdg << "  Name:" << cmnData<std::string >::HumanReadable(this->Name);
    return description__cdg.str();
}

prmSensorState::prmSensorState(void)
    : Name()
    , Values()
    , UserCount()
{}

prmSensorState::prmSensorState(const std::string &n)
    : Name(n)
    , Values()
    , UserCount()
{}

prmSensorState::prmSensorState(const prmSensorState & other):
      Name(other.Name)
    , Values(other.Values)
    , UserCount(other.UserCount)
{}

std::string prmSensorState::HumanReadable(void) const {
    std::stringstream description__cdg;
    description__cdg << "prmSensorState" << std::endl;
//    description__cdg << "  Name:" << cmnData<std::string >::HumanReadable(this->Name);
//    description__cdg << "  Values:" << cmnData<vctDynamicVector<double> >::HumanReadable(this->Values);
//    description__cdg << "  UserCount:" << cmnData<int >::HumanReadable(this->UserCount);
    return description__cdg.str();
}

prmSensorState::~prmSensorState(void){}

prmJointState::prmJointState(void):
      Position()
    , Velocity()
    , Acceleration()
{}

prmJointState::prmJointState(const prmJointState & other):
      Position(other.Position)
    , Velocity(other.Velocity)
    , Acceleration(other.Acceleration)
{}

prmJointState::~prmJointState(void){}

