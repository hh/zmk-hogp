# Copyright (c) 2024 The ZMK Contributors
# SPDX-License-Identifier: MIT

if(CONFIG_ZMK_HOGP OR CONFIG_ZMK_SERIAL_CMD)
  # Generate build version header at build time (consumed by serial_cmd.c)
  set(VERSION_HEADER ${CMAKE_CURRENT_BINARY_DIR}/zmk_build_version.h)
  add_custom_command(
      OUTPUT ${VERSION_HEADER}
      COMMAND ${CMAKE_COMMAND}
          -DGIT_DIR=${CMAKE_CURRENT_SOURCE_DIR}
          -DOUTPUT_FILE=${VERSION_HEADER}
          -P ${CMAKE_CURRENT_SOURCE_DIR}/src/version.cmake
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      COMMENT "Generating zmk-hogp build version header"
      VERBATIM
  )
  add_custom_target(zmk_hogp_build_version DEPENDS ${VERSION_HEADER})
  target_include_directories(app PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
  add_dependencies(app zmk_hogp_build_version)
endif()

if(CONFIG_ZMK_HOGP)
  target_sources(app PRIVATE src/hogp_central.c)
  target_include_directories(app PRIVATE include)
endif()

if(CONFIG_ZMK_HOGP_MOUSE_OUTPUT)
  target_sources(app PRIVATE src/hogp_mouse.c)
endif()

if(CONFIG_ZMK_HOGP_TRACKPAD_OUTPUT)
  target_sources(app PRIVATE src/hogp_trackpad.c)
endif()

if(CONFIG_ZMK_HOGP_ITRACK_OUTPUT)
  target_sources(app PRIVATE src/hogp_itrack.c)
endif()

if(CONFIG_ZMK_SERIAL_CMD)
  target_sources(app PRIVATE src/serial_cmd.c)
endif()
