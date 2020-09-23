cmake_minimum_required(VERSION 2.8.3)
project(ichthus_lidar_driver)

add_compile_options(-std=c++11)

find_package(PCL REQUIRED)
# find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
					roscpp
					sensor_msgs
					std_msgs
					pcl_ros
					pcl_conversions
					nodelet
					message_generation
	)

add_message_files(FILES 
	OSI64.msg 
	VLP16.msg
	)

generate_messages(DEPENDENCIES 
	std_msgs
	)

catkin_package(
				CATKIN_DEPENDS
				roscpp
        sensor_msgs
        std_msgs
        pcl_ros
        pcl_conversions
        nodelet
				message_runtime
	)


# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR NAMES yaml_cpp.h PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY NAMES YAML_CPP PATHS ${YAML_CPP_LIBRARY_DIRS})
link_directories(${YAML_CPP_LIBRARY_DIRS})
if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

###########################
### build configuration ###
###########################

include_directories(
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
	include
	)

add_library(ichthus_lidar_driver
		src/ichthus_lidar_driver/ichthus_midend.cpp
		src/ichthus_lidar_driver/ichthus_frontend.cpp
		src/ichthus_lidar_driver/ichthus_backend.cpp
		src/ichthus_lidar_driver/common/util.cpp
		src/ichthus_lidar_driver/common/calibration.cpp
		src/ichthus_lidar_driver/common/lidar.cpp
		src/ichthus_lidar_driver/common/packet.cpp
	)

set(libpcap_LIBRARIES "-lpcap")

find_package(CUDA)
# AW_CHECK_CUDA()
if (USE_CUDA)
    message("-- USING ACCELERATED TRANSFORM --")
    message("Version: " ${CUDA_VERSION})
    message("Library: " ${CUDA_CUDA_LIBRARY})
    message("Runtime: " ${CUDA_CUDART_LIBRARY})
		target_compile_definitions(ichthus_lidar_driver PRIVATE 
						USE_GPU=1)

    cuda_add_library(gpu_transform
						include/gpu_transform/gpu_transform.cuh
						# include/gpu_transform/gpu_transform.cu
						src/gpu_transform/gpu_transform.cu
            )

		target_link_libraries(ichthus_lidar_driver 
						${catkin_LIBRARIES} 
						${libpcap_LIBRARIES} 
						${YAML_CPP_LIBRARIES}
						gpu_transform
						)

    install(TARGETS
						gpu_transform
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )

else ()
		target_link_libraries(ichthus_lidar_driver 
						${catkin_LIBRARIES} 
						${libpcap_LIBRARIES} 
						${YAML_CPP_LIBRARIES}
						)

endif ()	

install(TARGETS ichthus_lidar_driver
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)

install(FILES ichthus_lidar_driver.xml 
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
	)

install(DIRECTORY cfg/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg
	)
