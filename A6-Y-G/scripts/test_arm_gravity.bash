#!/usr/bin/env bash
# Offline regression only. All serial integration tests create /dev/pts devices.
set -euo pipefail
A6Y_GRAVITY_TEST_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$A6Y_GRAVITY_TEST_ROOT/scripts/environment.bash"
source "$A6Y_GRAVITY_TEST_ROOT/scripts/build_gravity_stack.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${A6Y_GRAVITY_DOMAIN_ID:-197}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$A6Y_GRAVITY_TEST_ROOT/scripts/gravity_dds.xml}"
export ROS_LOG_DIR="$A6Y_GRAVITY_TEST_ROOT/../diagnostics/arm_gravity_test_logs"
mkdir -p "$ROS_LOG_DIR"
QT_QPA_PLATFORM=offscreen /usr/bin/python3 -m unittest discover -s "$A6Y_GRAVITY_TEST_ROOT/tools/arm_gravity" -p 'test_*.py' -v
/usr/bin/python3 -m unittest discover -s "$A6Y_GRAVITY_TEST_ROOT/tools/arm_diagnostics" -p 'test_operation_lock.py' -v
ctest --test-dir "$A6Y_GRAVITY_BUILD/build/a6_y_controllers" --output-on-failure -R 'gravity|friction|safe_zero|haptic|bounded'
ctest --test-dir "$A6Y_GRAVITY_BUILD/build/a6_y_ros2_control" --output-on-failure
/usr/bin/python3 "$A6Y_GRAVITY_TEST_ROOT/tools/arm_gravity/verify_gravity.py"
/usr/bin/python3 "$A6Y_GRAVITY_TEST_ROOT/tools/arm_gravity/verify_gravity.py" --live
/usr/bin/python3 "$A6Y_GRAVITY_TEST_ROOT/tools/arm_gravity/verify_gravity.py" --role teaching_left --fault
/usr/bin/python3 "$A6Y_GRAVITY_TEST_ROOT/tools/arm_gravity/verify_gravity.py" --role motion_left --readonly-close
/usr/bin/python3 "$A6Y_GRAVITY_TEST_ROOT/tools/arm_gravity/verify_gravity.py" --role teaching_right --cancel-enable
/usr/bin/python3 "$A6Y_GRAVITY_TEST_ROOT/tools/arm_gravity/verify_gravity.py" --role teaching_right --cancel-tuning
/usr/bin/python3 "$A6Y_GRAVITY_TEST_ROOT/tools/arm_gravity/verify_gravity.py" --role teaching_right --preenabled
/usr/bin/python3 "$A6Y_GRAVITY_TEST_ROOT/tools/arm_gravity/verify_multi.py" --roles teaching_left teaching_right motion_left motion_right
/usr/bin/python3 "$A6Y_GRAVITY_TEST_ROOT/tools/arm_gravity/verify_home.py"
/usr/bin/python3 "$A6Y_GRAVITY_TEST_ROOT/tools/arm_gravity/verify_home.py" --role teaching_right --scenario cancel
/usr/bin/python3 "$A6Y_GRAVITY_TEST_ROOT/tools/arm_gravity/verify_home.py" --role teaching_right --scenario plan_failure
/usr/bin/python3 "$A6Y_GRAVITY_TEST_ROOT/tools/arm_gravity/verify_home.py" --role teaching_right --scenario feedback_loss
/usr/bin/python3 "$A6Y_GRAVITY_TEST_ROOT/tools/arm_gravity/verify_home.py" --role teaching_right --scenario tracking_failure
QT_QPA_PLATFORM=offscreen /usr/bin/python3 "$A6Y_GRAVITY_TEST_ROOT/tools/arm_gravity/verify_ui.py" --output "$A6Y_GRAVITY_TEST_ROOT/../diagnostics/arm_gravity_ui_offscreen"
