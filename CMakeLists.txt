cmake_minimum_required(VERSION 3.5)
project(can_test)

# 查找 Eigen3 库
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# 添加包含目录 - 将 Eigen 路径放在最前面
include_directories(
    ${EIGEN3_INCLUDE_DIRS}  # 必须先添加 Eigen 路径
    include
    can
    ${PROJECT_SOURCE_DIR}/../Nuc_Dog/include  # 添加 Nuc_Dog 头文件路径
)

# 或者使用更现代的方式，针对特定目标设置
# include_directories(include can ${PROJECT_SOURCE_DIR}/../Nuc_Dog/include)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
link_libraries(pthread)
link_directories(${PROJECT_SOURCE_DIR}/can)

# CAN收发节点
add_executable(can_node
  src/Tangair_usb2can.cpp
  src/can_node.cpp
  src/dog_fsm.cpp
  src/gait_scheduler.cpp
  src/leg_kinematics.cpp
  src/leg_controller.cpp
  src/leg_swing.cpp
  src/state_passive.cpp
  src/state_stand.cpp
  src/state_trot.cpp
  src/state_jump.cpp
  src/rc.cpp
  src/KeyboardReader.cpp
  src/imu_data.c
  src/imu_reader.cpp
  src/radar_data.cpp
)

# 为目标单独添加包含目录（更推荐的方式）
target_include_directories(can_node PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
    include
    can
    ${PROJECT_SOURCE_DIR}/../Nuc_Dog/include
)

target_link_libraries(can_node 
    pthread
    usb_can
    ${CMAKE_THREAD_LIBS_INIT}  
    ${CMAKE_DL_LIBS}
)



