cmake_minimum_required(VERSION 3.8)
project(ur_description)

find_package(ament_cmake REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

install(DIRECTORY launch
	DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
	DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY meshes
	DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY cfg
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
