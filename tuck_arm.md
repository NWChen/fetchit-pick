rosnode cleanup
pgrep --full tuck_arm | xargs sudo kill -9
rosrun fetch_teleop tuck_arm.py -t
