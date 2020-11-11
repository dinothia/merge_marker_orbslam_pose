from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from utils.plot_helpers import *
from utils.read_files import *
import os


TRIP_NR = 4
deg2euler = np.pi/180

if __name__ == "__main__":
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
    start_time_in_s = 0
    duration_in_s = (t_slam[-1]-t_slam[0])

    start_idx_slam = np.round(start_time_in_s/dt_slam).astype("int")
    start_idx_gt = ((np.abs(t_gt - t_slam[start_idx_slam])).argmin() + gps_bag_delay_s / dt_gt).round().astype("int") # compensate for bag delays
    start_idx_marker = np.round(start_time_in_s/dt_marker).astype("int")

    end_idx_slam = start_idx_slam + np.round(duration_in_s/dt_slam).astype("int")    
    end_idx_gt = ((np.abs(t_gt - t_slam[end_idx_slam])).argmin() + gps_bag_delay_s / dt_gt).round().astype("int") # compensate for bag delays
    end_idx_marker = start_idx_marker + np.round(duration_in_s/dt_marker).astype("int")   
    
    t_gt, tvecs_gt, eulers_gt = t_gt[start_idx_gt:end_idx_gt], tvecs_gt[start_idx_gt:end_idx_gt,:], eulers_gt[start_idx_gt:end_idx_gt,:]
    t_slam, tvecs_slam, eulers_slam = t_slam[start_idx_slam:end_idx_slam], tvecs_slam[start_idx_slam:end_idx_slam,:], eulers_slam[start_idx_slam:end_idx_slam,:]
    t_marker, tvecs_marker, eulers_marker = t_marker[start_idx_marker:end_idx_marker], tvecs_marker[start_idx_marker:end_idx_marker,:], eulers_marker[start_idx_marker:end_idx_marker,:]

    # Reset all time 0 to zero
    t_gt -= t_gt[0]
    t_slam -= t_slam[0]
    t_marker -= t_marker[0]

    """
    tvecs_slam *= 104.75
    #Rot = R.from_euler('yx',[0,0], degrees=True).as_matrix() 
    #Rot2 = R.from_euler('zyx',0*eulers_gt[0], degrees=False).as_matrix() 
    #tvecs_slam = (Rot2 @ Rot @ tvecs_slam.T).T

    yaw_offset = 8.3+eulers_gt[0][2]*180/np.pi
    yaw_offset = 60
    Rot2 = R.from_euler('y', yaw_offset, degrees=True).as_matrix() 
    tvecs_slam = (Rot2 @ tvecs_slam.T).T
    tvecs_slam[:,2] += tvecs_gt[0,0] - tvecs_slam[0,2]
    tvecs_slam[:,0] += tvecs_gt[0,1] - tvecs_slam[0,0]

    plt.plot(t_gt, eulers_gt[:,2])
    plt.plot(t_slam, eulers_slam[:,1]+eulers_gt[0,2])
    plt.plot(t_marker, eulers_marker[:,1])
    plt.show()
    
    plt.figure(1)
    plt.plot(tvecs_gt[:,1],tvecs_gt[:,0])
    #plt.axis('scaled')
    #plt.figure(2)
    plt.plot(tvecs_slam[:,0],tvecs_slam[:,2])
    #plt.axis('scaled')
    plt.show()
    """
    """    
    plt.figure(1)
    plt.subplot(311)
    plt.plot(t_gt, tvecs_gt[:,0])
    plt.subplot(312)    
    plt.plot(t_gt, tvecs_gt[:,1])
    plt.subplot(313)    
    plt.plot(t_gt, tvecs_gt[:,2])
    plt.figure(2)
    plt.subplot(311)
    plt.plot(t_slam, tvecs_slam[:,0])
    plt.subplot(312)    
    plt.plot(t_slam, tvecs_slam[:,1])
    plt.subplot(313)    
    plt.plot(t_slam, tvecs_slam[:,2])
    plt.figure(3)
    plt.subplot(211)
    plt.plot(tvecs_gt[:,1], tvecs_gt[:,0])
    plt.subplot(212)
    plt.plot(-tvecs_slam[:,1], tvecs_slam[:,0])

   
    plt.figure(4)
    plt.subplot(311)
    plt.plot(t_slam, tvecs_slam[:,0])
    plt.subplot(312)    
    plt.plot(t_slam, tvecs_slam[:,1])
    plt.subplot(313)    
    plt.plot(t_slam, tvecs_slam[:,2])
    plt.figure(5)
    Rot = R.from_euler('yx',[90,90], degrees=True).as_matrix() 
    tvecs_slam = (Rot @ tvecs_slam.T).T
    plt.subplot(311)
    plt.plot(t_slam, tvecs_slam[:,0])
    plt.subplot(312)    
    plt.plot(t_slam, tvecs_slam[:,1])
    plt.subplot(313)    
    plt.plot(t_slam, tvecs_slam[:,2])

    """


    plt.show()

    """
    ################################
    ## Transfprm all to NED frame ##
    ################################
    camera_yaw_offset = 8.3 * deg2euler

    y_rot = eulers_gt[0,2] 
    x_rot = eulers_gt[0,1]
    z_rot = eulers_gt[0,0]

    #R_slam2ned1 = R.from_euler('zyx',[z_rot,y_rot,x_rot]).as_matrix()
    #R_slam2ned2 = R.from_euler('zyx',[-90,y_rot,-90], degrees=True).as_matrix()
    
    #R_slam2ned = R_slam2ned2 @ R_slam2ned1
    
    tvecs_temp = tvecs_slam.copy()
   
    #tvecs_slam *= 197.9326/16.95400

    #tvecs_slam += tvecs_gt[0,:]

    #eulers_slam = (R_slam2ned @ eulers_slam.T).T

    
    plt.figure(1)
    plot_eulers(t_gt, eulers_gt)
    plot_eulers(t_slam, eulers_slam)
    plt.plot(t_slam, eulers_slam[:,1])

    plt.figure(2)
    plot_tvecs(t_gt, tvecs_gt)
    plot_tvecs(t_slam, tvecs_slam)

    plt.figure(3)
    plt.plot(t_gt, tvecs_gt[:,0])

    R_slam2ned = R.from_euler('y',64, degrees=True).as_matrix()
    tvecs_slam = (R_slam2ned @ tvecs_temp.copy().T).T
    tvecs_slam *= 197.9326/16.95400
    tvecs_slam[:,2] += tvecs_gt[0,0]

    plt.plot(t_slam, tvecs_slam[:,2])

    R_slam2ned = R.from_euler('y',0, degrees=True).as_matrix()
    tvecs_slam = (R_slam2ned @ tvecs_temp.copy().T).T
    tvecs_slam *= 197.9326/16.95400
    tvecs_slam[:,2] += tvecs_gt[0,0]

    plt.plot(t_slam, tvecs_slam[:,2])

    plt.show()
    """