cd /home/arcsystem/catkin_ws
for iter in $(seq 1 20); do
	xdg-open /home/arcsystem/Desktop/Rollout/ROutMOVIE.mp4
	sleep 1.0
	roslaunch arc_presentation lift_presentation.launch
	sleep 10.0
	killall totem
done