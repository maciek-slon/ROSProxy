function (USE_ROS)
    if (NOT DEFINED ENV{ROS_DISTRO})
        MESSAGE(FATAL_ERROR "ROS not sourced")
    else()
        SET(ROS_DISTRO $ENV{ROS_DISTRO})
        MESSAGE(STATUS "ROS distro: ${ROS_DISTRO}")
    endif()

    # Cannot use ARGN directly with list() command.
    # Copy to a variable first.
    set (extra_macro_args ${ARGN})

    # Did we get any optional args?
    list(LENGTH extra_macro_args num_extra_args)
    if (${num_extra_args} GREATER 0)
        MESSAGE(STATUS "Using ROS packages:")
        foreach(loop_var ${extra_macro_args})
            message(STATUS "${loop_var}")
            FIND_PACKAGE(${loop_var})
            set(INT_ROS_LIBRARIES ${INT_ROS_LIBRARIES} ${${loop_var}_LIBRARIES})
            include_directories(${${loop_var}_INCLUDE_DIRS})
        endforeach()
    endif()

    set(ROS_LIBRARIES ${ROS_LIBRARIES} ${INT_ROS_LIBRARIES} PARENT_SCOPE)
endfunction ()
