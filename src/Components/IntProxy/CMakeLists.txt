# Include the directory itself as a path to include directories
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

USE_ROS(roscpp std_msgs)

# Create an executable file from sources:
ADD_LIBRARY(IntProxy SHARED ${files})

# Link external libraries
TARGET_LINK_LIBRARIES(IntProxy ${DisCODe_LIBRARIES} ${ROS_LIBRARIES})

INSTALL_COMPONENT(IntProxy)
