cmake_minimum_required(VERSION 2.8.3)
project(ik_solver)

find_package(catkin REQUIRED COMPONENTS
  iiwa_msgs
  geometry_msgs
  roscpp
  message_generation
)

add_message_files(DIRECTORY msg)
generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime
  geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

set(MAIN_SOURCES
    # basic
    src/kinematics/homogenous.cpp
    # ros communication + kuka specific
    src/kuka/ros_iiwa_link.cpp
    src/kuka/iiwa_kinematics.cpp
    # ros communication + ur specific
    src/ur/ur_link.cpp
    src/ur/ur_kinematics.cpp
    # finite state machine
    #src/fsm/collision_detection.cpp
    #src/fsm/collision_reactions.cpp
    #src/fsm/main_task.cpp
    #src/fsm/task_dispatcher.cpp
    # src/fsm/torque_particle_filter.cpp
    #src/fsm/collision_localization.cpp
)

# kuka iiwa solver
add_executable(iiwa_calc src/iiwa_calc.cpp ${MAIN_SOURCES})
add_dependencies(iiwa_calc iiwa_msgs_generate_messages_cpp)
add_dependencies(iiwa_calc ik_solver_generate_messages_cpp)
target_link_libraries(iiwa_calc ${catkin_LIBRARIES})

# ur solver
add_executable(ur_calc src/ur_calc.cpp ${MAIN_SOURCES})
add_dependencies(ur_calc iiwa_msgs_generate_messages_cpp)
add_dependencies(ur_calc ik_solver_generate_messages_cpp)
target_link_libraries(ur_calc ${catkin_LIBRARIES})
