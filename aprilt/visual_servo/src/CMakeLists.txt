project(tutorial-ros-pioneer)

cmake_minimum_required(VERSION 3.0.2)

find_package(visp_ros REQUIRED
             visual_servo)

include_directories( "~/vip_ws/src/visual_servo/include/"
                    ${visp_ros_INCLUDE_DIRS})

add_executable(tutorial-ros-pioneer-visual-servo tutorial-ros-pioneer-visual-servo.cpp)
target_link_libraries(tutorial-ros-pioneer-visual-servo ${visp_ros_LIBRARIES})

add_executable(tutorial-ros-pioneer-visual-servo-4points tutorial-ros-pioneer-visual-servo-4points.cpp)
target_link_libraries(tutorial-ros-pioneer-visual-servo-4points ${visp_ros_LIBRARIES})

add_executable(qr_track qr_track.cpp)
target_link_libraries(qr_track ${visp_ros_LIBRARIES} -lzbar -lm)

add_executable(qr qr.cpp)
target_link_libraries(qr ${visp_ros_LIBRARIES} -lzbar -lm)


add_executable(test_qr test.cpp)
target_link_libraries(test_qr ${visp_ros_LIBRARIES} -lzbar -lm)

add_executable(qqq qqq.cpp)
target_link_libraries(qqq ${visp_ros_LIBRARIES} -lzbar -lm)

add_executable(4point 4point.cpp)
target_link_libraries(4point ${visp_ros_LIBRARIES} -lzbar -lm)