# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# add the component directories that we want to use
set(EXTRA_COMPONENT_DIRS
  "components/"
  "components/espp/components/"
)

set(
  COMPONENTS
  "main esptool_py cli filters bldc_haptics motorgo-mini tinys3_test_stand"
  CACHE STRING
  "List of components to include"
  )

project(bldc_test_stand)

set(CMAKE_CXX_STANDARD 20)
