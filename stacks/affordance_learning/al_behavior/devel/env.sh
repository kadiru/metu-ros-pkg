#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/user/metu-ros-pkg/affordance_learning/al_behavior/devel', type 'exit' to leave"
  . "/home/user/metu-ros-pkg/affordance_learning/al_behavior/devel/setup.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/user/metu-ros-pkg/affordance_learning/al_behavior/devel'"
else
  . "/home/user/metu-ros-pkg/affordance_learning/al_behavior/devel/setup.sh"
  exec "$@"
fi
