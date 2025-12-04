# Copyright (c) 2024 The ZMK Contributors
# SPDX-License-Identifier: MIT

if(CONFIG_ZMK_HOGP)
  target_sources(app PRIVATE src/hogp_central.c)
  target_include_directories(app PRIVATE include)
endif()

if(CONFIG_ZMK_HOGP_MOUSE_OUTPUT)
  target_sources(app PRIVATE src/hogp_mouse.c)
endif()

if(CONFIG_ZMK_SERIAL_CMD)
  target_sources(app PRIVATE src/serial_cmd.c)
endif()
