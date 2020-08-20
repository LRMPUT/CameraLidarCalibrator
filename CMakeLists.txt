cmake_minimum_required(VERSION 2.8.3)
project(camera_lidar_calibrator)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	  roscpp
	  rospy
	  std_msgs
	  geometry_msgs
	  sensor_msgs
        roslib
        pcl_ros
        tf2
        tf2_ros
        cv_bridge
        image_transport
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

set(G20_LIBRARIES  ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_calibration_odom_laser.so
		${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_solver_pcg.so
		${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_cli.so
		${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_solver_slam2d_linear.so
		${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_core.so
		${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_solver_structure_only.so
		${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_csparse_extension.so
		${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_stuff.so
		${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_ext_freeglut_minimal.so
		${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_tutorial_slam2d.so
		${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_hierarchical.so
		${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_types_data.so
		${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_incremental.so
		${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_types_icp.so
		${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_interactive.so
		${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_types_sba.so
		${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_interface.so
		${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_types_sclam2d.so
		${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_opengl_helper.so
		${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_types_sim3.so
		${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_parser.so
		${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_types_slam2d_addons.so
		${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_simulator.so
		${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_types_slam2d.so
		${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_solver_cholmod.so
		${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_types_slam3d_addons.so
		${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_solver_csparse.so
		${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_types_slam3d.so
		${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_solver_dense.so)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES laser_image_publisher
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs tf2_ros tf2 pcl_ros
  DEPENDS EIGEN3 PCL OpenCV
)

###########
## Build ##
###########

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o
  ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/build
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
		src/local_g2o/edge_se3_time.cpp
		src/local_g2o/edge_one_prior.cpp
		src/local_g2o/vertex_one.cpp
		src/local_g2o/register_types.cpp
		src/local_g2o/minimal_plane.cpp
		src/utils/CustomVoxelGrid.cpp
        src/utils/BSpline.cpp
		src/Optimization.cpp
        src/detections/CameraDetection.cpp
        src/detections/LaserDetection.cpp
		src/CameraLaserCalibration.cpp)

target_link_libraries(${PROJECT_NAME}
		${catkin_LIBRARIES}
		${PCL_LIBRARIES}
		${OpenCV_LIBS}
		${G20_LIBRARIES}
		)

#### Possible calibrations

# Calibration based on rosbag
add_executable(${PROJECT_NAME}_node
		examples/rosbag/CameraLaserCalibrationBAG.cpp
		examples/rosbag/calibrate_rosbag.cpp)
target_link_libraries(${PROJECT_NAME}_node
   ${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${G20_LIBRARIES}
 )

# Calibration based on simulation
add_executable(${PROJECT_NAME}_sim
		examples/simulation/CameraLaserCalibrationSIM.cpp
		examples/simulation/calibrate_simulation.cpp)
target_link_libraries(${PROJECT_NAME}_sim
		${PROJECT_NAME}
		${catkin_LIBRARIES}
		${PCL_LIBRARIES}
		${OpenCV_LIBS}
		${G20_LIBRARIES}
		)