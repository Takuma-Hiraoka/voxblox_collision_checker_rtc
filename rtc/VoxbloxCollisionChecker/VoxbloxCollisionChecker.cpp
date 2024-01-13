#include "VoxbloxCollisionChecker.h"
#include <cnoid/BodyLoader>

VoxbloxCollisionChecker::VoxbloxCollisionChecker(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_qIn_("qIn", m_q_),
  m_basePosIn_("basePosIn", m_basePos_),
  m_baseRpyIn_("baseRpyIn", m_baseRpy_),
  m_fieldTransformIn_("fieldTransformIn", m_fieldTransform_),
  m_tsdfmapIn_("voxbloxLayerIn", m_tsdfmap_),
  m_collisionOut_("collisionOut", m_collision_)
{
}

RTC::ReturnCode_t VoxbloxCollisionChecker::onInitialize(){
  std::cerr << "[" << this->m_profile.instance_name << "] onInitialize()" << std::endl;

  addInPort("qIn", this->m_qIn_);
  addInPort("basePosIn", this->m_basePosIn_);
  addInPort("baseRpyIn", this->m_baseRpyIn_);
  addInPort("fieldTransformIn", this->m_fieldTransformIn_);
  addInPort("voxbloxLayerIn", this->m_tsdfmapIn_);
  addOutPort("collisionOut", this->m_collisionOut_);

  // load robot model
  cnoid::BodyLoader bodyLoader;
  std::string fileName;
  if(this->getProperties().hasKey("model")) fileName = std::string(this->getProperties()["model"]);
  else fileName = std::string(this->m_pManager->getConfig()["model"]); // 引数 -o で与えたプロパティを捕捉
  if (fileName.find("file://") == 0) fileName.erase(0, strlen("file://"));
  std::cerr << "[" << this->m_profile.instance_name << "] model: " << fileName <<std::endl;
  this->robot_ = bodyLoader.load(fileName);
  if(!this->robot_){
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
    return RTC::RTC_ERROR;
  }

  return RTC::RTC_OK;
}

RTC::ReturnCode_t VoxbloxCollisionChecker::onExecute(RTC::UniqueId ec_id){
  return RTC::RTC_OK;
}

static const char* VoxbloxCollisionChecker_spec[] = {
  "implementation_id", "VoxbloxCollisionChecker",
  "type_name",         "VoxbloxCollisionChecker",
  "description",       "VoxbloxCollisionChecker component",
  "version",           "0.0",
  "vendor",            "Takume-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};

extern "C"{
    void VoxbloxCollisionCheckerInit(RTC::Manager* manager) {
        RTC::Properties profile(VoxbloxCollisionChecker_spec);
        manager->registerFactory(profile, RTC::Create<VoxbloxCollisionChecker>, RTC::Delete<VoxbloxCollisionChecker>);
    }
};
