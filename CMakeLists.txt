cmake_minimum_required(VERSION 3.5)
project(variator_driver)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

find_package(catkin REQUIRED COMPONENTS roscpp hardware_interface controller_manager)

catkin_package(CATKIN_DEPENDS roscpp)

include_directories(${catkin_INCLUDE_DIRS} include)

# for ethercat grand
set(GCC_NEWDTAGS_LINK_FLAGS "-Wl,--disable-new-dtags")
set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} ${GCC_NEWDTAGS_LINK_FLAGS}")

add_executable(
  ${PROJECT_NAME} src/variator_ros_driver.cpp src/variator_ros_driver_node.cpp
                  include/motor_imitator/motor_client_imitation.cpp
                 )

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
