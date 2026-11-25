cmake_minimum_required(VERSION 3.8)
project(remora_controller)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  src/pattern_status_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pattern_status_publisher
)

install(PROGRAMS
  src/pattern_supervisor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pattern_supervisor
)

install(PROGRAMS
  src/gnss_range_sensor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gnss_range_sensor
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
