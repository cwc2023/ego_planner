# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "dynamic_reconfigure;quadrotor_msgs;roscpp;rospy;geometry_msgs;sensor_msgs;mavros_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lpx4ctrl".split(';') if "-lpx4ctrl" != "" else []
PROJECT_NAME = "px4ctrl"
PROJECT_SPACE_DIR = "/home/cwc/ego_planner/install"
PROJECT_VERSION = "0.0.0"
