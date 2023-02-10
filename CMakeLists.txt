cmake_minimum_required(VERSION 3.5)
project(csi_driver)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(class_loader REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)
ament_export_include_directories(include)

add_executable(csi_driver_node src/csi_driver.cpp include/csi_driver/csi_driver.hpp)
ament_target_dependencies(csi_driver_node
        rclcpp
        rclcpp_components
        image_transport
        cv_bridge
        OpenCV
        std_msgs
        sensor_msgs)

add_library(csi_driver_component
        SHARED src/csi_driver.cpp)
target_compile_definitions(csi_driver_component
        PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(csi_driver_component
        rclcpp
        rclcpp_components
        image_transport
        cv_bridge
        OpenCV
        std_msgs
        sensor_msgs)
rclcpp_components_register_nodes(csi_driver_component
        "csi_driver::CSIDriverNode")

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

install(DIRECTORY include
        DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})
install(TARGETS csi_driver_node
        DESTINATION lib/${PROJECT_NAME})
install(TARGETS csi_driver_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

ament_package()
