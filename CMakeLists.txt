cmake_minimum_required(VERSION 3.10)

# プロジェクト名
project(manipulator_planner)

# C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Eigenライブラリのパスを指定
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(OpenCV REQUIRED)
# # Pythonのパッケージを探す
# find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
# find_package(Python3 REQUIRED COMPONENTS NumPy)

# # Pythonのインクルードディレクトリを追加
# include_directories(${Python3_INCLUDE_DIRS})
# include_directories(${Python3_NumPy_INCLUDE_DIRS})

# # 実行ファイルの出力先を設定
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

# # 実行ファイルを生成
# add_executable(main_app src/main.cpp)

# # Eigenライブラリをリンク
# target_link_libraries(main_app Eigen3::Eigen ${Python3_LIBRARIES})


set(DEPEND_ROS_PKGS
  roscpp
  tf2_ros
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  visualization_msgs
  grid_map_msgs
  grid_map_ros
  cv_bridge
)

find_package(catkin REQUIRED
  COMPONENTS
  ${DEPEND_ROS_PKGS}
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS
 ${DEPEND_ROS_PKGS}
 DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(manipulator_planner_node src/manipulator_planner_node.cpp)
add_dependencies(manipulator_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(manipulator_planner_node ${catkin_LIBRARIES} Eigen3::Eigen ${OpenCV_LIBRARIES})
