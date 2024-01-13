#include "VoxbloxCollisionChecker.h"
#include <cnoid/BodyLoader>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshExtractor>
#include <cnoid/EigenUtil>

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
    std::vector<cnoid::Vector3f> vertices; // 同じvertexが2回カウントされている TODO
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
        cnoid::Vector3f v0 = mesh->vertices()->at(mesh->triangle(j)[0]);
        cnoid::Vector3f v1 = mesh->vertices()->at(mesh->triangle(j)[1]);
        cnoid::Vector3f v2 = mesh->vertices()->at(mesh->triangle(j)[2]);
        float l1 = (v1 - v0).norm();
        float l2 = (v2 - v0).norm();
        cnoid::Vector3f n1 = (v1 - v0).normalized();
        cnoid::Vector3f n2 = (v2 - v0).normalized();
        for(double m=0;m<l1;m+=resolution){
          for(double n=0;n<l2-l2/l1*m;n+=resolution){
            cnoid::Vector3f v = v0 + n1 * m + n2 * n;
            int x = int((v[0] - bbx.min()[0])/resolution);
            int y = int((v[1] - bbx.min()[1])/resolution);
            int z = int((v[2] - bbx.min()[2])/resolution);
            if(!bin[x][y][z]){
              bin[x][y][z] = true;
              vertices.push_back(v);
            }
          }
          double n=l2-l2/l1*m;
          cnoid::Vector3f v = v0 + n1 * m + n2 * n;
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
        cnoid::Vector3f v = v0 + n1 * m + n2 * n;
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
