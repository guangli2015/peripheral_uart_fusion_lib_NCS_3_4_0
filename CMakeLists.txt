#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(NONE)

# NORDIC SDK APP START
target_sources(app PRIVATE
  src/main.c
  src/Fusion_1.3.3/Fusion/FusionAhrs.c
  src/Fusion_1.3.3/Fusion/FusionBias.c
  src/Fusion_1.3.3/Fusion/FusionCompass.c
  src/Fusion_1.3.3/Fusion/FusionConvention.c
  src/Fusion_1.3.3/Fusion/FusionRemap.c
)

target_include_directories(app PRIVATE
  src/Fusion_1.3.3/Fusion
)

# NORDIC SDK APP END
