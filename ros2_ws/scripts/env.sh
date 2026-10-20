#!/usr/bin/env bash
# Source this file in each ROS/Isaac terminal. Do not execute it.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo "Use: source scripts/env.sh" >&2
    exit 1
fi

_xlerobot_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo "ROS 2 Jazzy is required at /opt/ros/jazzy." >&2
    unset _xlerobot_ws
    return 1
fi
source /opt/ros/jazzy/setup.bash
if [[ -d "$_xlerobot_ws/.deps/python" ]]; then
    export PYTHONPATH="$_xlerobot_ws/.deps/python${PYTHONPATH:+:$PYTHONPATH}"
fi
_xlerobot_deps="$_xlerobot_ws/.deps/root"
_xlerobot_prefix="$_xlerobot_deps/opt/ros/jazzy"
if [[ -d "$_xlerobot_prefix" ]]; then
    export AMENT_PREFIX_PATH="$_xlerobot_prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
    export CMAKE_PREFIX_PATH="$_xlerobot_prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
    export LD_LIBRARY_PATH="$_xlerobot_prefix/lib:$_xlerobot_deps/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export LIBRARY_PATH="$_xlerobot_prefix/lib:$_xlerobot_deps/usr/lib/x86_64-linux-gnu${LIBRARY_PATH:+:$LIBRARY_PATH}"
    export PYTHONPATH="$_xlerobot_prefix/lib/python3.12/site-packages:$_xlerobot_deps/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
    export PATH="$_xlerobot_prefix/bin:$PATH"
fi
# NVIDIA dependencies are extracted locally as well. Never add CUDA's driver
# stubs: libcuda must resolve to the actual host graphics driver.
for _xlerobot_lib in \
    "$_xlerobot_deps/usr/local/cuda-13.0/targets/x86_64-linux/lib" \
    "$_xlerobot_deps/opt/nvidia/vpi4/lib/x86_64-linux-gnu" \
    "$_xlerobot_prefix/lib/x86_64-linux-gnu" \
    "$_xlerobot_prefix"/share/*/gxf/lib \
    "$_xlerobot_prefix"/share/isaac_ros_gxf/gxf/lib/*; do
    if [[ -d "$_xlerobot_lib" ]]; then
        export LD_LIBRARY_PATH="$_xlerobot_lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    fi
done
unset _xlerobot_lib
if [[ -f "$_xlerobot_ws/install/local_setup.bash" ]]; then
    source "$_xlerobot_ws/install/local_setup.bash"
fi
export ISAAC_SIM_PATH="${ISAAC_SIM_PATH:-$HOME/isaacsim}"
export XLEROBOT_ASSET="${XLEROBOT_ASSET:-$_xlerobot_ws/../xlerobot_isaac_sim/xlerobot_dd.usd}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
# Use the same domain in all terminals; keep this simulation separate from domain 0.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$_xlerobot_ws/log/ros}"
case ":$PATH:" in
    *":$_xlerobot_ws/scripts:"*) ;;
    *) export PATH="$_xlerobot_ws/scripts:$PATH" ;;
esac
unset _xlerobot_ws
unset _xlerobot_deps _xlerobot_prefix
