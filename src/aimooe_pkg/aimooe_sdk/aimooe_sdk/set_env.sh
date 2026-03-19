#!/bin/bash

module_path=~/zzrobot_ws/src/aimooe
lib_path=~/zzrobot_ws/src/aimooe/aimooeSDK

export PYTHONPATH=$module_path:$PYTHONPATH
echo "add $module_path to PYTHONPATH"

export LD_LIBRARY_PATH=$lib_path:$LD_LIBRARY_PATH
echo "add $lib_path to LD_LIBRARY_PATH"