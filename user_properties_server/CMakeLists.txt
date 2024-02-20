cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/bluetooth/esp_ble_mesh/common_components/example_init)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(user_properties_server)
