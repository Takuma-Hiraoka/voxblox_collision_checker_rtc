#ifndef VOXBLOXCOLLISIONCHECKER_H
#define VOXBLOXCOLLISIONCHECKER_H

#include <unordered_map>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <cnoid/Body>

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>

#include <collision_checker_msgs/idl/Collision.hh>
#include <voxblox_msgs_rtmros_bridge/idl/Voxblox.hh>

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

 private:
  cnoid::BodyPtr robot_;

  std::unordered_map<cnoid::LinkPtr, std::vector<cnoid::Vector3f> > verticesMap_;

  std::vector<cnoid::LinkPtr> targetLinks_;
};

extern "C"
{
  void VoxbloxCollisionCheckerInit(RTC::Manager* manager);
};


#endif // VOXBLOXCOLLISIONCHECKER_H
