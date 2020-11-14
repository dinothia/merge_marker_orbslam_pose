import numpy as np
from utils.angle_conversion import *
import scipy.io


def read_pose_orbslam(filepath):
    t_list = []
    tvecs_list = []
    euler_list = []

    tvecs = np.zeros((3, len(tvecs_list)))

    with open(filepath, "r") as file:
        for line in file:
            current_line = line.split() 
            
            t = np.array(current_line[0]).astype('float')
            tvec = np.array(current_line[1:4]).astype('float')
            q = np.array(current_line[4:8]).astype('float')
            euler = quaternion_to_euler(q[0], q[1], q[2], q[3])

            t_list.append(t)
            tvecs_list.append(tvec)
            euler_list.append(euler)
        
        t = np.array(t_list).astype('float') 
        tvecs = np.array(tvecs_list).astype('float')
        eulers = np.array(euler_list).astype('float')
    
    return t, tvecs, eulers

def read_pose_marker(filepath):
    t_list = []
    tvecs_list = []
    euler_list = []

    tvecs = np.zeros((3, len(tvecs_list)))

    with open(filepath, "r") as file:
        for line in file:
            current_line = line.split(",") 
            
            t = np.array(current_line[0]).astype('float')
            tvec = np.array(current_line[1:4]).astype('float')
            rvec = np.array(current_line[4:7]).astype('float')
            euler = np.array(rvec)

            t_list.append(t)
            tvecs_list.append(tvec)
            euler_list.append(euler)
        
        t = np.array(t_list).astype('float')
        tvecs = np.array(tvecs_list).astype('float')
        eulers = np.array(euler_list).astype('float')
    
    return t, tvecs, eulers


def read_ground_truth(filepath):
    ground_truth = scipy.io.loadmat(filepath)
    
    t = ground_truth["obsv_estimates"][0][0][0]
    navigaton_frame = ground_truth["obsv_estimates"][0][0][1]
    roll_hat = ground_truth["obsv_estimates"][0][0][2]
    pitch_hat = ground_truth["obsv_estimates"][0][0][3]
    yaw_hat = ground_truth["obsv_estimates"][0][0][4]
    
    omega_ib_b_hat =  ground_truth["obsv_estimates"][0][0][5]
    ars_bias_hat = ground_truth["obsv_estimates"][0][0][6]
    ars_bias_total_hat = ground_truth["obsv_estimates"][0][0][7] 
    acc_bias_hat = ground_truth["obsv_estimates"][0][0][8] 
    gravity_hat = ground_truth["obsv_estimates"][0][0][9] 
    tmo_innovation = ground_truth["obsv_estimates"][0][0][10] 
    T_tmo_innovation = ground_truth["obsv_estimates"][0][0][11] 
    T_tmo_innovation_sum = ground_truth["obsv_estimates"][0][0][12] 

    p_Lb_L_hat = ground_truth["obsv_estimates"][0][0][13] 
    
    v_eb_n_hat = ground_truth["obsv_estimates"][0][0][14] 
    speed_course_hat = ground_truth["obsv_estimates"][0][0][15] 
    innov = ground_truth["obsv_estimates"][0][0][16] 
    innov_covariance = ground_truth["obsv_estimates"][0][0][17] 
    P_hat = ground_truth["obsv_estimates"][0][0][18] 

    t = np.array(t.T).astype('float')
    tvecs = p_Lb_L_hat.copy().T.astype('float')

    eulers = np.zeros((len(t), 3)).astype('float')
    eulers[:,0] = roll_hat.copy()
    eulers[:,1] = pitch_hat.copy()
    eulers[:,2] = yaw_hat.copy()

    return t, tvecs, eulers, v_eb_n_hat.T

def read_all3_day2_trip_paths(trip_nr):
    ground_truth_path = f"/home/dino/Datasets/Kaia/Ground Truth/obsv_estimates ({trip_nr}).mat"
    orbslam_pose_path = "/home/dino/Installs/ORB_SLAM3/KeyFrameTrajectory.txt"
    marker_pose_path = "/home/dino/Documents/Prosjektoppgave/Code/charuco_pose_estimation/cameraTrajectory.txt"
    
    gps_bag_delay_s = [18.24,18.33,18.42,18.54]  # delay between RTK-GPS time (4) and rosbag trondheim 4 time
    
    return ground_truth_path, orbslam_pose_path, marker_pose_path, gps_bag_delay_s[trip_nr-1]