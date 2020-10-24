## Script for running mono orbslam and marker detection at the same time
# SLAM
gnome-terminal --tab -- bash -c "roscore"
gnome-terminal --tab -- bash -c "cd ~/Installs/ORB_SLAM3 && rosrun ORB_SLAM3 Mono Vocabulary/ORBvoc.txt Examples/Monocular/Kaia.yaml"
gnome-terminal --tab -- bash -c "sleep 2 && rosrun image_transport republish compressed in:=/camera/image_raw raw out:=/camera/image_raw"

# Marker
gnome-terminal --tab -- bash -c "sleep 1 && python3 ~/Documents/Prosjektoppgave/Code/charuco_pose_estimation/main.py /camera/image_raw/compressed ~/Documents/Prosjektoppgave/Code/charuco_pose_estimation/cameraParams.yaml ~/Documents/Prosjektoppgave/Code/charuco_pose_estimation/cameraTrajectory.txt" 

# Run bag file
gnome-terminal --tab -- bash -c "sleep 10 && rosbag play -r 0.5 ~/Datasets/Kaia/trondheim2_inn.bag -s 800"
