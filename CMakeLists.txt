cmake_minimum_required(VERSION 2.8.3)
project(pal_debug_tts)

find_package(catkin REQUIRED)

catkin_package()

install(
  PROGRAMS
    scripts/pal_debug_tts.py
    scripts/pal_speak.py
    scripts/pal_topic_tts.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
