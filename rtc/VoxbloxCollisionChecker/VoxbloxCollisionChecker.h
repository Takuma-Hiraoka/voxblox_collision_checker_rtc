#ifndef VOXBLOXCOLLISIONCHECKER_H
#define VOXBLOXCOLLISIONCHECKER_H

#include <unordered_map>
#include <memory>
#include <thread>
#include <mutex>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <cnoid/Body>

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>

#include <collision_checker_msgs/idl/Collision.hh>
#include <voxblox_msgs_rtmros_bridge/idl/Voxblox.hh>

#include "voxblox/core/esdf_map.h"
#include "voxblox/integrator/esdf_integrator.h"

class VoxbloxCollisionChecker : public RTC::DataFlowComponentBase
{
 protected:
  RTC::TimedDoubleSeq m_q_;
  RTC::InPort<RTC::TimedDoubleSeq> m_qIn_;
  RTC::TimedPoint3D m_basePos_;
  RTC::InPort<RTC::TimedPoint3D> m_basePosIn_;
  RTC::TimedOrientation3D m_baseRpy_;
  RTC::InPort<RTC::TimedOrientation3D> m_baseRpyIn_;
  RTC::TimedPose3D m_fieldTransform_;
  RTC::InPort<RTC::TimedPose3D> m_fieldTransformIn_;
  voxblox_msgs_rtmros_bridge::TimedVoxbloxLayer m_tsdfmap_;
  RTC::InPort<voxblox_msgs_rtmros_bridge::TimedVoxbloxLayer> m_tsdfmapIn_;

  collision_checker_msgs::TimedCollisionSeq m_collision_;
  RTC::OutPort<collision_checker_msgs::TimedCollisionSeq> m_collisionOut_;

 public:
  VoxbloxCollisionChecker(RTC::Manager* manager);

  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
  void voxbloxCallback(std::shared_ptr<voxblox::Layer<voxblox::TsdfVoxel>> tsdf_layer, voxblox::EsdfMap::Config esdf_config);

 private:
  std::mutex mutex_;

  cnoid::BodyPtr robot_;

  std::shared_ptr<std::thread> thread_;
  bool thread_done_ = true;

  std::unordered_map<cnoid::LinkPtr, std::vector<cnoid::Vector3> > verticesMap_;

  std::vector<cnoid::LinkPtr> targetLinks_;

  cnoid::Position fieldOrigin_;

  std::shared_ptr<voxblox::EsdfMap> esdfMap_ = nullptr;
  std::shared_ptr<voxblox::EsdfIntegrator> esdfIntegrator_ = nullptr;
  double minWeight_ = 1e-6;
  double maxDistance_ = 1.0;
  double minDistance_ = -0.02;
  double defaultDistance_ = 1.0;
};

extern "C"
{
  void VoxbloxCollisionCheckerInit(RTC::Manager* manager);
};


#endif // VOXBLOXCOLLISIONCHECKER_H
