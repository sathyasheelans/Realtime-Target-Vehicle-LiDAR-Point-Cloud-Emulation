#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/santhanam.17/Carla_scripts/catkin_ws/src/geometry2_python3/tf2_sensor_msgs"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/santhanam.17/Carla_scripts/catkin_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/santhanam.17/Carla_scripts/catkin_ws/install/lib/python3/dist-packages:/home/santhanam.17/Carla_scripts/catkin_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/santhanam.17/Carla_scripts/catkin_ws/build" \
    "/usr/bin/python3" \
    "/home/santhanam.17/Carla_scripts/catkin_ws/src/geometry2_python3/tf2_sensor_msgs/setup.py" \
     \
    build --build-base "/home/santhanam.17/Carla_scripts/catkin_ws/build/geometry2_python3/tf2_sensor_msgs" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/santhanam.17/Carla_scripts/catkin_ws/install" --install-scripts="/home/santhanam.17/Carla_scripts/catkin_ws/install/bin"
