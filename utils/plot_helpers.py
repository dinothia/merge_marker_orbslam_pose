import matplotlib.pyplot as plt


def plot_tvecs(t, tvecs):
    plt.subplot(311)
    plt.plot(t, tvecs[:,0])    
    
    plt.subplot(312)
    plt.plot(t, tvecs[:,1])

    plt.subplot(313)
    plt.plot(t, tvecs[:,2])


def plot_eulers(t, eulers):
    plt.subplot(311)
    plt.plot(t, eulers[:,0])    
    
    plt.subplot(312)
    plt.plot(t, eulers[:,1])

    plt.subplot(313)
    plt.plot(t, eulers[:,2])