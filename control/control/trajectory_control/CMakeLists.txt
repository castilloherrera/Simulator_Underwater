cmake_minimum_required(VERSION 2.8.3)
project(trajectory_control)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS scripts/auv_geometric_tracking_controller_circle.py
			       scripts/auv_geometric_tracking_controller_rect.py
			       scripts/auv_geometric_tracking_controller_Npolygon.py
                               scripts/auv_geometric_tracking_controller_rect_file.py
			       scripts/auv_geometric_tracking_controller_path_random_file.py
			       scripts/auv_geometric_tracking_controller_bluerov2_circle.py
			       scripts/auv_geometric_tracking_controller_bluerov2_rect_file.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

