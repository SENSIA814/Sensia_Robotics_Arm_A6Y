#!/usr/bin/env bash
# Offline only. No physical serial port is opened.
set -euo pipefail
A6Y_TEACHING_TEST="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$A6Y_TEACHING_TEST/scripts/environment.bash"
source "$A6Y_TEACHING_TEST/scripts/build_gravity_stack.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=196
export CYCLONEDDS_URI="file://$A6Y_TEACHING_TEST/scripts/gravity_dds.xml"
export ROS_LOG_DIR="$A6Y_TEACHING_TEST/../diagnostics/arm_teaching_test_logs"
mkdir -p "$ROS_LOG_DIR"
QT_QPA_PLATFORM=offscreen /usr/bin/python3 -m unittest discover -s "$A6Y_TEACHING_TEST/tools/arm_gravity" -p 'test_*.py' -v
ctest --test-dir "$A6Y_GRAVITY_BUILD/build/a6_y_controllers" --output-on-failure -R 'a5_gravity|gravity_home|bounded|safe_zero|friction|haptic'
ctest --test-dir "$A6Y_GRAVITY_BUILD/build/a6_y_ros2_control" --output-on-failure
/usr/bin/python3 "$A6Y_TEACHING_TEST/tools/arm_gravity/verify_teaching_port.py"
/usr/bin/python3 "$A6Y_TEACHING_TEST/tools/arm_gravity/verify_gripper_haptics.py"
/usr/bin/python3 "$A6Y_TEACHING_TEST/tools/arm_gravity/verify_teaching_port.py" --single --exit-failure
/usr/bin/python3 "$A6Y_TEACHING_TEST/tools/arm_gravity/verify_teaching_port.py" --single --fault
/usr/bin/python3 "$A6Y_TEACHING_TEST/tools/arm_gravity/verify_teaching_port.py" --single --mode-failure
/usr/bin/python3 "$A6Y_TEACHING_TEST/tools/arm_gravity/verify_teaching_port.py" --single --cancel-home
QT_QPA_PLATFORM=offscreen /usr/bin/python3 "$A6Y_TEACHING_TEST/tools/arm_gravity/verify_teaching_ui.py"
