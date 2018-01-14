
template<typename VFT, typename DT> void mtsVFController::SetVF(const DT &vf)
{
  if (!SetVFData(vf))
  {
    VFMap[vf.Name] = new VFT(vf.Name, new DT(vf));
    IncrementUsers(vf.KinNames, vf.SensorNames);
  }
}

template<typename DT> bool mtsVFController::SetVFData(const DT &data)
{
  std::map<std::string, mtsVFBase*>::iterator itVF;
  itVF = VFMap.find(data.Name);

  if (itVF == VFMap.end())
  {
    return false;
  }

  DecrementUsers(itVF->second->Data->KinNames, itVF->second->Data->SensorNames);
  delete itVF->second->Data;
  itVF->second->Data = new DT(data);
  IncrementUsers(itVF->second->Data->KinNames, itVF->second->Data->SensorNames);
  return true;
}

template<typename KT> void mtsVFController::SetKinematics(const KT &kin)
{
  RemoveKinematicsFromMap(kin.Name);
  Kinematics.insert(std::pair<std::string, prmKinematicsState *>(kin.Name, new KT(kin)));
}
