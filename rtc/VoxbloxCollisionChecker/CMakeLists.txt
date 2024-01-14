add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild
  collision_checker_msgs
  voxblox
  voxblox_msgs_rtmros_bridge
  )
find_package(choreonoid REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  )

# 相対パスを絶対パスに直す
set(CHOREONOID_BODY_LIBRARIES_ABS)
foreach(lib ${CHOREONOID_BODY_LIBRARIES})
  find_library(${lib}_abs NAMES ${lib} PATHS ${CHOREONOID_LIBRARY_DIRS})
  set(CHOREONOID_BODY_LIBRARIES_ABS ${CHOREONOID_BODY_LIBRARIES_ABS} ${${lib}_abs})
endforeach(lib)

rtmbuild_add_library(VoxbloxCollisionChecker SHARED
  VoxbloxCollisionChecker.cpp
  )
target_link_libraries(VoxbloxCollisionChecker
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES_ABS}
  )
set_target_properties(VoxbloxCollisionChecker PROPERTIES PREFIX "") # libVoxbloxCollisionChecker.so -> VoxbloxCollisionChecker.so
add_dependencies(VoxbloxCollisionChecker RTMBUILD_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl

rtmbuild_add_executable(VoxbloxCollisionCheckerComp VoxbloxCollisionCheckerComp.cpp)
target_link_libraries(VoxbloxCollisionCheckerComp VoxbloxCollisionChecker)
