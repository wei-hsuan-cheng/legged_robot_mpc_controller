#!/bin/bash
# Arm-free Pineapple stand-up assets, using the shared contact-aware TO tools.
# Same commands as config/pineapple_manipulator/regenerate.sh; separate models, candidates and export.
export ROBOT=pineapple
exec bash "$(dirname "$0")/../pineapple_manipulator/regenerate.sh" "$@"
