import matplotlib.pyplot as plt
import numpy as np


def plot_tvecs(t, tvecs, w_dots=False):
    plt.subplot(311)
    plt.plot(t, tvecs[:,0], marker='.' if w_dots else '')      
    plt.ylabel("x [m]")
    
    plt.subplot(312)
    plt.plot(t, tvecs[:,1], marker='.' if w_dots else '')    
    plt.ylabel("y [m]")

    plt.subplot(313)
    plt.plot(t, tvecs[:,2], marker='.' if w_dots else '')    
    plt.ylabel("z [m]")
    plt.xlabel("time [s]")

def plot_eulers(t, eulers, w_dots=False):
    plt.subplot(311)
    plt.plot(t, eulers[:,0], marker='.' if w_dots else '')    
    plt.ylabel("roll [deg]")
    
    plt.subplot(312)
    plt.plot(t, eulers[:,1], marker='.' if w_dots else '')    
    plt.ylabel("pitch [deg]")

    plt.subplot(313)
    plt.plot(t, eulers[:,2], marker='.' if w_dots else '')    
    plt.ylabel("yaw [deg]")
    plt.xlabel("time [s]")

def scatter_eulers(t, eulers):
    plt.subplot(311)
    plt.scatter(t, eulers[:,0], c='g')    
    plt.ylabel("roll [deg]")
    
    plt.subplot(312)
    plt.scatter(t, eulers[:,1], c='g')       
    plt.ylabel("pitch [deg]")

    plt.subplot(313)
    plt.scatter(t, eulers[:,2], c='g')         
    plt.ylabel("yaw [deg]")
    plt.xlabel("time [s]")

def scatter_tvecs(t, tvecs):
    plt.subplot(311)
    plt.scatter(t, tvecs[:,0], c='g')    
    plt.ylabel("x [m]")
    
    plt.subplot(312)
    plt.scatter(t, tvecs[:,1], c='g')       
    plt.ylabel("y [m]")

    plt.subplot(313)
    plt.scatter(t, tvecs[:,2], c='g')         
    plt.ylabel("z [m]")
    plt.xlabel("time [s]")


def plot_set_lim_eulers(t):
    plt.subplot(311)
    plt.xlim([t[0], t[-1]]) 
    
    plt.subplot(312)
    plt.xlim([t[0], t[-1]]) 

    plt.subplot(313)
    plt.xlim([t[0], t[-1]]) 

def plot_set_legend(legend1, legend2):
    plt.subplot(311)
    plt.legend([legend1, legend2])
    
    plt.subplot(312)
    plt.legend([legend1, legend2])

    plt.subplot(313)
    plt.legend([legend1, legend2])


def plot_tvecs_xy(tvec1, tvec2):
    plt.subplot(311)
    plt.plot(tvec1[:,0], tvec1[:,1])
    plt.plot(tvec2[:,0], tvec2[:,1])
    plt.ylabel("y [m]")
    plt.xlabel("x [m]")
    
    plt.subplot(312)
    plt.plot(tvec1[:,0], tvec1[:,2])
    plt.plot(tvec2[:,0], tvec2[:,2])
    plt.ylabel("z [m]")
    plt.xlabel("x [m]")

    plt.subplot(313)
    plt.plot(tvec1[:,1], tvec1[:,2])
    plt.plot(tvec2[:,1], tvec2[:,2])
    plt.ylabel("z [m]")
    plt.xlabel("y [m]")