#include "VoxbloxCollisionChecker.h"
#include <cnoid/BodyLoader>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshExtractor>
#include <cnoid/EigenUtil>
#include <cnoid/TimeMeasure>

#include "voxblox/core/block.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"

static void addMesh(cnoid::SgMeshPtr model, std::shared_ptr<cnoid::MeshExtractor> meshExtractor){
  cnoid::SgMeshPtr mesh = meshExtractor->currentMesh();
  const cnoid::Affine3& T = meshExtractor->currentTransform();

  const int vertexIndexTop = model->getOrCreateVertices()->size();

  const cnoid::SgVertexArray& vertices = *mesh->vertices();
  const int numVertices = vertices.size();
  for(int i=0; i < numVertices; ++i){
    const cnoid::Vector3 v = T * vertices[i].cast<cnoid::Affine3::Scalar>();
    model->vertices()->push_back(v.cast<cnoid::Vector3f::Scalar>());
  }

  const int numTriangles = mesh->numTriangles();
  for(int i=0; i < numTriangles; ++i){
    cnoid::SgMesh::TriangleRef tri = mesh->triangle(i);
    const int v0 = vertexIndexTop + tri[0];
    const int v1 = vertexIndexTop + tri[1];
    const int v2 = vertexIndexTop + tri[2];
    model->addTriangle(v0, v1, v2);
  }
}

static cnoid::SgMeshPtr convertToSgMesh (const cnoid::SgNodePtr collisionshape){
  if (!collisionshape) return nullptr;

  std::shared_ptr<cnoid::MeshExtractor> meshExtractor = std::make_shared<cnoid::MeshExtractor>();
  cnoid::SgMeshPtr model = new cnoid::SgMesh;
  if(meshExtractor->extract(collisionshape, [&]() { addMesh(model,meshExtractor); })){
    model->setName(collisionshape->name());
  }else{
    std::cerr << "[convertToSgMesh] meshExtractor->extract failed " << collisionshape->name() << std::endl;
    return nullptr;
  }

  return model;
}

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

  // get link vertices
  float resolution = 0.01;
  for(int i=0;i<robot_->numLinks();i++){
    cnoid::LinkPtr link = robot_->link(i);
    std::vector<cnoid::Vector3> vertices; // 同じvertexが2回カウントされている TODO
    cnoid::SgMeshPtr mesh = convertToSgMesh(link->collisionShape());
    if(mesh) {
      mesh->updateBoundingBox();
      cnoid::BoundingBoxf bbx = mesh->boundingBox();
      cnoid::Vector3f bbxSize = bbx.max() - bbx.min();
      std::vector<std::vector<std::vector<bool> > > bin(int(bbxSize[0]/resolution)+1,
                                                        std::vector<std::vector<bool> >(int(bbxSize[1]/resolution)+1,
                                                                                        std::vector<bool>(int(bbxSize[2]/resolution)+1,
                                                                                                          false)));

      for(int j=0;j<mesh->numTriangles();j++){
        cnoid::Vector3 v0 = mesh->vertices()->at(mesh->triangle(j)[0]).cast<cnoid::Vector3d::Scalar>();
        cnoid::Vector3 v1 = mesh->vertices()->at(mesh->triangle(j)[1]).cast<cnoid::Vector3d::Scalar>();
        cnoid::Vector3 v2 = mesh->vertices()->at(mesh->triangle(j)[2]).cast<cnoid::Vector3d::Scalar>();
        double l1 = (v1 - v0).norm();
        double l2 = (v2 - v0).norm();
        cnoid::Vector3 n1 = (v1 - v0).normalized();
        cnoid::Vector3 n2 = (v2 - v0).normalized();
        for(double m=0;m<l1;m+=resolution){
          for(double n=0;n<l2-l2/l1*m;n+=resolution){
            cnoid::Vector3 v = v0 + n1 * m + n2 * n;
            int x = int((v[0] - bbx.min()[0])/resolution);
            int y = int((v[1] - bbx.min()[1])/resolution);
            int z = int((v[2] - bbx.min()[2])/resolution);
            if(!bin[x][y][z]){
              bin[x][y][z] = true;
              vertices.push_back(v);
            }
          }
          double n=l2-l2/l1*m;
          cnoid::Vector3 v = v0 + n1 * m + n2 * n;
          int x = int((v[0] - bbx.min()[0])/resolution);
          int y = int((v[1] - bbx.min()[1])/resolution);
          int z = int((v[2] - bbx.min()[2])/resolution);
          if(!bin[x][y][z]){
            bin[x][y][z] = true;
            vertices.push_back(v);
          }
        }
        double m = l1;
        double n= 0;
        cnoid::Vector3 v = v0 + n1 * m + n2 * n;
        int x = int((v[0] - bbx.min()[0])/resolution);
        int y = int((v[1] - bbx.min()[1])/resolution);
        int z = int((v[2] - bbx.min()[2])/resolution);
        if(!bin[x][y][z]){
          bin[x][y][z] = true;
          vertices.push_back(v);
        }
      }
    }
    verticesMap_[link] = vertices;
  }

  for(int i=0;i<robot_->numLinks();i++){
    cnoid::LinkPtr link = robot_->link(i);
    if(verticesMap_[link].size() == 0) continue;
    targetLinks_.push_back(link);
  }

  return RTC::RTC_OK;
}

RTC::ReturnCode_t VoxbloxCollisionChecker::onExecute(RTC::UniqueId ec_id){

  std::lock_guard<std::mutex> guard(this->mutex_);

  if (this->thread_done_ && this->thread_){
    this->thread_->join();
    this->thread_ = nullptr;
  }

  if (this->m_qIn_.isNew()) this->m_qIn_.read();
  if (this->m_basePosIn_.isNew()) this->m_basePosIn_.read();
  if (this->m_baseRpyIn_.isNew()) this->m_baseRpyIn_.read();

  if(this->m_q_.data.length() == this->robot_->numJoints()){
    for ( int i = 0; i < this->robot_->numJoints(); i++ ){
      this->robot_->joint(i)->q() = this->m_q_.data[i];
    }
  }
  this->robot_->rootLink()->p()[0] = m_basePos_.data.x;
  this->robot_->rootLink()->p()[1] = m_basePos_.data.y;
  this->robot_->rootLink()->p()[2] = m_basePos_.data.z;
  this->robot_->rootLink()->R() = cnoid::rotFromRpy(m_baseRpy_.data.r, m_baseRpy_.data.p, m_baseRpy_.data.y);
  this->robot_->calcForwardKinematics();

  if (this->m_fieldTransformIn_.isNew()) {
    this->m_fieldTransformIn_.read();
    this->fieldOrigin_.translation()[0] = m_fieldTransform_.data.position.x;
    this->fieldOrigin_.translation()[1] = m_fieldTransform_.data.position.y;
    this->fieldOrigin_.translation()[2] = m_fieldTransform_.data.position.z;
    this->fieldOrigin_.linear() = cnoid::rotFromRpy(m_fieldTransform_.data.orientation.r, m_fieldTransform_.data.orientation.p, m_fieldTransform_.data.orientation.y);
  }

  std::vector<collision_checker_msgs::CollisionIdl> collisions;

  if (this->m_tsdfmapIn_.isNew()) {
    this->m_tsdfmapIn_.read();
    voxblox::Layer<voxblox::TsdfVoxel>::Ptr tsdf_layer = std::make_shared<voxblox::Layer<voxblox::TsdfVoxel>>(m_tsdfmap_.data.voxel_size, m_tsdfmap_.data.voxels_per_side);

    // convert tsdf_layer
    {
      // layer_type
      // action
      for (int i=0;i<m_tsdfmap_.data.blocks.length();i++) {
        voxblox::BlockIndex index(m_tsdfmap_.data.blocks[i].x_index, m_tsdfmap_.data.blocks[i].y_index, m_tsdfmap_.data.blocks[i].z_index);
        // see voxblox_ros/conversions_inl.h
        typename voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
          tsdf_layer->allocateBlockPtrByIndex(index);

        std::vector<uint32_t> data(m_tsdfmap_.data.blocks[i].data.length());
        for (int j=0; j<data.size(); j++) {
          data[j] = m_tsdfmap_.data.blocks[i].data[j];
        }
        block_ptr->deserializeFromIntegers(data);
      }
      if ( !this->thread_) {
	voxblox::EsdfMap::Config esdf_config;
	// Same voxel size for ESDF as with TSDF
	esdf_config.esdf_voxel_size = m_tsdfmap_.data.voxel_size;
	// Same number of voxels per side for ESDF as with TSDF
	esdf_config.esdf_voxels_per_side = m_tsdfmap_.data.voxels_per_side;
	this->thread_done_ = false;
	this->thread_ = std::make_shared<std::thread>(&VoxbloxCollisionChecker::voxbloxCallback, this, tsdf_layer, esdf_config);
      }
    }
  }
  // collision
  {
    if (this->esdfMap_) {
      std::shared_ptr<voxblox::EsdfMap> esdfMap = this->esdfMap_;
      Eigen::Affine3d fieldOriginInv = this->fieldOrigin_.inverse();
      for(int i=0;i<this->targetLinks_.size();i++){
	cnoid::LinkPtr link = this->targetLinks_[i];

	double min_dist = this->maxDistance_ + 1;
	cnoid::Vector3 closest_v = cnoid::Vector3::Zero();
	cnoid::Vector3 closest_point_fieldLocal = cnoid::Vector3::Zero();
	cnoid::Vector3 closest_direction_fieldLocal = cnoid::Vector3::UnitX();

	const std::vector<cnoid::Vector3>& vertices = this->verticesMap_[link];
	for(int j=0;j<vertices.size();j++){
	  cnoid::Vector3 v = link->T() * vertices[j];

	  cnoid::Vector3 v_fieldLocal = fieldOriginInv * v;

	  cnoid::Vector3 grad;
	  double dist;
	  esdfMap->getDistanceAndGradientAtPosition(v_fieldLocal, &dist, &grad);
	  if(grad.norm() > 0){
	    if(dist < min_dist){
	      closest_direction_fieldLocal[0] = (grad[0]/grad.norm());
	      closest_direction_fieldLocal[1] = (grad[1]/grad.norm());
	      closest_direction_fieldLocal[2] = (grad[2]/grad.norm());
	      closest_point_fieldLocal[0] = v_fieldLocal[0]-closest_direction_fieldLocal[0]*dist;
	      closest_point_fieldLocal[1] = v_fieldLocal[1]-closest_direction_fieldLocal[1]*dist;
	      closest_point_fieldLocal[2] = v_fieldLocal[2]-closest_direction_fieldLocal[2]*dist;
	      min_dist = dist;
	      closest_v = vertices[j];
	    }
	  }
	}

	if(min_dist <= this->maxDistance_ && min_dist >= this->minDistance_){
	  cnoid::Vector3 closest_point = this->fieldOrigin_ * closest_point_fieldLocal;
	  cnoid::Vector3 closest_direction = this->fieldOrigin_.linear() * closest_direction_fieldLocal;

	  collision_checker_msgs::CollisionIdl collision;
	  collision.link1 = link->name().c_str();
	  collision.point1.x = closest_v[0];
	  collision.point1.y = closest_v[1];
	  collision.point1.z = closest_v[2];
	  collision.link2 = "";
	  collision.point2.x = closest_point[0];
	  collision.point2.y = closest_point[1];
	  collision.point2.z = closest_point[2];
	  collision.direction21.x = closest_direction[0];
	  collision.direction21.y = closest_direction[1];
	  collision.direction21.z = closest_direction[2];
	  collision.distance = min_dist;
	  collisions.push_back(collision);
	}
      } 
    }
  }

  this->m_collision_.tm = this->m_q_.tm;
  this->m_collision_.data.length(collisions.size());
  for(size_t i=0;i<collisions.size();i++){
    this->m_collision_.data[i] = collisions[i];
  }
  this->m_collisionOut_.write();

  return RTC::RTC_OK;
}

void VoxbloxCollisionChecker::voxbloxCallback(std::shared_ptr<voxblox::Layer<voxblox::TsdfVoxel>> tsdf_layer, voxblox::EsdfMap::Config esdf_config)
{
  cnoid::TimeMeasure timer;
  timer.begin();
  this->esdfMap_ = std::make_shared<voxblox::EsdfMap>(esdf_config);
  voxblox::EsdfIntegrator::Config esdf_integrator_config;
  esdf_integrator_config.min_weight = this->minWeight_;
  esdf_integrator_config.min_distance_m = this->minDistance_;
  esdf_integrator_config.max_distance_m = this->maxDistance_;;
  esdf_integrator_config.default_distance_m = this->defaultDistance_;
  esdf_integrator_config.collision_radius = 1.2;
  cnoid::Vector3d collisionOrigin_ = this->fieldOrigin_.inverse() * this->robot_->rootLink()->p();
  esdf_integrator_config.collision_origin = voxblox::Point(collisionOrigin_[0], collisionOrigin_[1], collisionOrigin_[2]);
  this->esdfIntegrator_ = std::make_shared<voxblox::EsdfIntegrator>(esdf_integrator_config, tsdf_layer.get(),
								    this->esdfMap_->getEsdfLayerPtr());
  this->esdfIntegrator_->updateFromTsdfLayerBatch();
  std::cerr << timer.measure() << std::endl;
  this->thread_done_ = true;
};

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
