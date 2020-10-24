import matplotlib.pyplot as plt
import numpy as np
import math


def euler_to_quaternion(roll, pitch, yaw):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return np.array([roll, pitch, yaw])

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


if __name__ == "__main__":    
    marker_pose_path = "/home/dino/Documents/Prosjektoppgave/Code/charuco_pose_estimation/cameraTrajectory.txt"
    orbslam_pose_path = "/home/dino/Installs/ORB_SLAM3/KeyFrameTrajectory.txt"

    t_Marker, tvecs_Marker, eulers_Marker = read_pose_marker(marker_pose_path)
    t_SLAM, tvecs_SLAM, eulers_SLAM = read_pose_orbslam(orbslam_pose_path)
    
    scale = 96.41 
    offset = 0

    plt.figure(1)
    plt.subplot(211)
    plt.title("Marker Z")
    plt.plot(t_Marker, tvecs_Marker[:,2])
    plt.subplot(212)
    plt.title("SLAM Y")
    plt.plot(t_SLAM, scale*tvecs_SLAM[:,1]+offset)


    plt.figure(2)
    plt.subplot(211)
    plt.title("Marker Y")
    plt.plot(t_Marker, tvecs_Marker[:,1])
    plt.subplot(212)
    plt.title("SLAM X")
    plt.plot(t_SLAM, scale*tvecs_SLAM[:,0]+offset)

    plt.figure(3)
    plt.subplot(211)
    plt.title("Marker X")
    plt.plot(t_Marker, tvecs_Marker[:,0])
    plt.subplot(212)
    plt.title("SLAM Z")
    plt.plot(t_SLAM, scale*tvecs_SLAM[:,2]+offset)

    plt.show()