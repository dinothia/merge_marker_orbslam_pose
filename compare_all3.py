from scipy.spatial.transform import Rotation as R
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from utils.plot_helpers import *
from utils.read_files import *
import os


TRIP_NR = 3
start_time_in_s = 0#1.8#13


if __name__ == "__main__":
    print("Check if trip number is correct")
    # Scipt to run all 3 pose estimations
    #script_path = "~/Documents/Prosjektoppgave/Code/merge_marker_orbslam_pose/run_marker_orbslam.sh "
    #s.system(f"{script_path} trondheim{TRIP_NR}_inn.bag 850 0.25")

    ###########################
    ## Read trajectory files ##
    ###########################
    (
        ground_truth_path,
        orbslam_pose_path,
        marker_pose_path,
        gps_bag_delay_s
    ) = read_all3_day2_trip_paths(TRIP_NR)

    t_gt, tvecs_gt, eulers_gt = read_ground_truth(ground_truth_path)    
    t_slam, tvecs_slam, eulers_slam = read_pose_orbslam(orbslam_pose_path)
    t_marker, tvecs_marker, eulers_marker = read_pose_marker(marker_pose_path)

    # fix marker bag time0 using slam time
    marker_bag_delay_s = (t_slam[-1]-t_slam[0]) - (t_marker[-1]-t_marker[0])
    t_marker += marker_bag_delay_s

    # Define sampling times
    dt_gt = 0.032
    dt_slam = np.mean(np.diff(t_slam))
    dt_marker = 0.1

    #####################################
    ## Trunc all 3 to same time length ##
    #####################################
    duration_in_s = (t_slam[-1]-t_slam[0]) - start_time_in_s

    start_idx_slam = np.round(start_time_in_s/dt_slam).astype("int")
    start_idx_gt = ((np.abs(t_gt - t_slam[start_idx_slam])).argmin() + gps_bag_delay_s / dt_gt).round().astype("int") # compensate for bag delays
    start_idx_marker = np.round(start_time_in_s/dt_marker).astype("int")

    end_idx_slam = start_idx_slam + np.round(duration_in_s/dt_slam).astype("int")    
    end_idx_gt = ((np.abs(t_gt - t_slam[end_idx_slam])).argmin() + gps_bag_delay_s / dt_gt).round().astype("int") # compensate for bag delays
    end_idx_marker = start_idx_marker + np.round(duration_in_s/dt_marker).astype("int")   
    
    t_gt, tvecs_gt, eulers_gt = t_gt[start_idx_gt:end_idx_gt], tvecs_gt[start_idx_gt:end_idx_gt,:], eulers_gt[start_idx_gt:end_idx_gt,:]
    t_slam, tvecs_slam, eulers_slam = t_slam[start_idx_slam:end_idx_slam], tvecs_slam[start_idx_slam:end_idx_slam,:], eulers_slam[start_idx_slam:end_idx_slam,:]
    t_marker, tvecs_marker, eulers_marker = t_marker[start_idx_marker:end_idx_marker], tvecs_marker[start_idx_marker:end_idx_marker,:], eulers_marker[start_idx_marker:end_idx_marker,:]

    # Set all time 0 to zero
    t_gt -= t_gt[0]
    t_slam -= t_slam[0]
    t_marker -= t_marker[0]

    # Set position time 0 to 0
    tvecs_slam -= tvecs_slam[0,:]
    eulers_slam -= eulers_slam[0,:]

    #############################
    ## SLAM 2 NED frame Angles ##
    #############################

    deg2rad = np.pi/180
    rad2deg = 180/np.pi

    R_cam2ned = R.from_euler('yx',[90,90], degrees=True).as_matrix() 
    eulers_slam = (R_cam2ned @ eulers_slam.T).T
    # Add euler offsets
    eulers_slam += eulers_gt[0,:]

    ###############################
    ## SLAM 2 NED frame Position ##
    ###############################
    # camera offset + ground truth offset
    roll_offset =   0.823   + eulers_gt[0,0]*rad2deg
    pitch_offset = (-2.807) + eulers_gt[0,1]*rad2deg
    yaw_offset =    8.3     + eulers_gt[0,2]*rad2deg

    # In camera frame, compensate for camera and ground truth (GT) offset
    Rot2 = R.from_euler('zyx', [roll_offset, yaw_offset, pitch_offset], degrees=True).as_matrix() 
    tvecs_slam = (Rot2 @ tvecs_slam.T).T

    # Camera frame to ned frame
    tvecs_slam_f = tvecs_slam.copy()
    tvecs_slam_f[:,0] = tvecs_slam[:,2]
    tvecs_slam_f[:,1] = tvecs_slam[:,0]
    tvecs_slam_f[:,2] = tvecs_slam[:,1]

    # Calculate scaling from orslam to GT and scale tvecs_slam
    scale_vector = ((np.max(tvecs_gt,0)-np.min(tvecs_gt,0))/(np.max(tvecs_slam_f,0)-np.min(tvecs_slam_f,0)))
    scale = np.mean(scale_vector[:2])
    tvecs_slam_f *= scale

    # Add tvecs ground truth offset
    tvecs_slam_f += tvecs_gt[0,:]

    ################################
    ## Benchmark info calculation ##
    ################################
    # angle between trajectories calculation in xy-plane 
    vector_1 = (tvecs_gt[-1,:]-tvecs_gt[0,:])[:2]
    vector_2 = (tvecs_slam_f[-1,:]-tvecs_slam_f[0,:])[:2]
    angle = calc_angle_between_vectors(vector_1, vector_2) * rad2deg    

    # max distance between trajectories in xy-plane
    max_xy_dist = np.linalg.norm(tvecs_gt[-1,0:2]-tvecs_slam_f[-1,0:2])

    ##########
    ## Plot ##
    ##########
    plt.figure()
    plot_tvecs(t_gt, tvecs_gt)
    plot_tvecs(t_slam, tvecs_slam_f, True)
    plot_set_lim_eulers(t_slam)
    plot_set_legend("Ground Truth (w. RTK)", "ORBSLAM3")
    plt.tight_layout()

    plt.figure()
    plot_eulers(t_gt, eulers_gt*rad2deg)
    plot_eulers(t_slam, eulers_slam*rad2deg, True)
    plot_set_lim_eulers(t_slam)
    plot_set_legend("Ground Truth (w. RTK)", "ORBSLAM3")
    plt.tight_layout()

    plt.figure()
    plot_tvecs_xy(tvecs_gt, tvecs_slam_f)
    plot_set_legend("Ground Truth (w. RTK)", "ORBSLAM3")
    plt.tight_layout()

    plt.figure()
    plt.plot(tvecs_gt[:,0], tvecs_gt[:,1])
    plt.plot(tvecs_slam_f[:,0], tvecs_slam_f[:,1])
    plt.title(f"Max xy-distance: {np.round(max_xy_dist,2)}m; Angle: {np.round(angle,2)}deg")
    plt.ylabel("y [m]")
    plt.xlabel("x [m]")
    plt.axis("scaled")
    plt.legend(["Ground Truth (w. RTK)", "ORBSLAM3"])
    plt.tight_layout()

    plt.show()
