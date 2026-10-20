#!/usr/bin/env bash
# Initialize the vendor Python environment, then replace the shell so ROS launch
# signals reach Isaac itself (python.sh otherwise leaves a waiting shell parent).
set -e
isaac_sim_dir="${1:?Isaac Sim installation directory is required}"
shift
SCRIPT_DIR="$isaac_sim_dir"
export CARB_APP_PATH="$isaac_sim_dir/kit"
export ISAAC_PATH="$isaac_sim_dir"
export EXP_PATH="$isaac_sim_dir/apps"
source "$isaac_sim_dir/setup_python_env.sh"
# Native ros2_control loads ROS plugins after startup. Their bundled dependencies
# must already be on the dynamic loader's search path when Python starts.
ros_distro="${ROS_DISTRO:-jazzy}"
ros_control="$isaac_sim_dir/exts/isaacsim.ros2.control/$ros_distro"
ros_core="$isaac_sim_dir/exts/isaacsim.ros2.core/$ros_distro"
if [[ ! -d "$ros_control/lib" || ! -d "$ros_core/lib" ]]; then
    echo "Isaac Sim 6.2 with isaacsim.ros2.control ($ros_distro) is required." >&2
    exit 1
fi
export LD_LIBRARY_PATH="$ros_control/lib:$ros_core/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export AMENT_PREFIX_PATH="$ros_control:$ros_core${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
# Isaac 6.2 ships its CUDA runtime here. Resolve it before the older system
# libcudart, which cannot load the bundled PyTorch library.
cuda_runtime="$isaac_sim_dir/exts/isaacsim.pip.nv/pip_prebundle/nvidia/cuda_runtime/lib"
if [[ -d "$cuda_runtime" ]]; then
    export LD_LIBRARY_PATH="$cuda_runtime${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
export RESOURCE_NAME=IsaacSim
export LD_PRELOAD="$isaac_sim_dir/kit/libcarb.so${LD_PRELOAD:+:$LD_PRELOAD}"
exec "${PYTHONEXE:-$isaac_sim_dir/kit/python/bin/python3}" "$@"
