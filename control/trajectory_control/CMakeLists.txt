cmake_minimum_required(VERSION 2.8.3)
project(trajectory_control)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS scripts/rov_nl_pid_controller.py
                               scripts/rov_mb_sm_controller.py
                               scripts/rov_nmb_sm_controller.py
                               scripts/rov_pid_controller.py
                               scripts/rov_ua_pid_controller.py
                               scripts/rov_mb_fl_controller.py
                               scripts/auv_geometric_tracking_controller.py
                               scripts/demo_wp_trajectory_generator.py
                               scripts/rov_pd_grav_compensation_controller.py
                               scripts/rov_sf_controller.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)


