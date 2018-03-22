from os import listdir
import numpy as np
import matplotlib.pylab as plt 

_cam = 1
_plane = 1


def list_files(cam, plane):
    path = "../hist/cam" + str(cam) + "/plane" + str(plane) + "/"
    return [path + file for file in listdir(path)]


def read_array(file_reader, n_lines):
    return [[int(i) for i in file_reader.readline().split()] for _ in range(n_lines)]


def read_file(path):
    with open(path, 'r') as f:
        f.readline()
        hist = read_array(f, 256)
    return np.array(hist)


def __main__():
    files = list_files(_cam, _plane)
    hist = read_file(files[0])
    plot_colors = ['r', 'g', 'b']
    for i in range(3):
        color = hist[:,i]
        plt.plot(np.linspace(0, 255, len(color)), color, plot_colors[i])s