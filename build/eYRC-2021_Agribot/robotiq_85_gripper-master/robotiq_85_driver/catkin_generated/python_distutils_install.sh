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

echo_and_run cd "/home/sanhita/catkin_ws/src/eYRC-2021_Agribot/robotiq_85_gripper-master/robotiq_85_driver"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/sanhita/catkin_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/sanhita/catkin_ws/install/lib/python3/dist-packages:/home/sanhita/catkin_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/sanhita/catkin_ws/build" \
    "/usr/bin/python3" \
    "/home/sanhita/catkin_ws/src/eYRC-2021_Agribot/robotiq_85_gripper-master/robotiq_85_driver/setup.py" \
     \
    build --build-base "/home/sanhita/catkin_ws/build/eYRC-2021_Agribot/robotiq_85_gripper-master/robotiq_85_driver" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/sanhita/catkin_ws/install" --install-scripts="/home/sanhita/catkin_ws/install/bin"
