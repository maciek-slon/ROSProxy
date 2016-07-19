# Include the directory itself as a path to include directories
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

# Find required packages
FIND_PACKAGE( OpenCV REQUIRED )

USE_ROS(roscpp std_msgs sensor_msgs image_transport cv_bridge message_filters)

# Create an executable file from sources:
ADD_LIBRARY(KinectSubscriber SHARED ${files})

# Link external libraries
TARGET_LINK_LIBRARIES(KinectSubscriber ${DisCODe_LIBRARIES} 
	${ROS_LIBRARIES}
	${OpenCV_LIBS})

INSTALL_COMPONENT(KinectSubscriber)
