
import matplotlib.pyplot as plt
from utilities import FileReader

import math
import numpy as np


def plot_errors(filename): 

    headers, values=FileReader(filename).read_file()
    

    plt.plot([lin[11] for lin in values], [lin[12] for lin in values], label= "kf")

    i = 0
    for lin in values:
        if (i%50 == 0 and i<1000):

            list = find_ellipse(lin[14], lin[15], lin[11], lin[12], lin[13])
            plt.plot([x[0] for x in list], [x[1] for x in list], '-r')

        i += 1

    plt.title("state space")
    plt.grid()
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.legend()
    plt.xlim([-2, 15])
    plt.ylim([-2, 15])

    plt.show()

import argparse

def find_ellipse(var_x, var_y, x, y, th, scale=800):
    t = np.linspace(0, 2*math.pi, 100)
    sd_x = math.sqrt(var_x) / scale
    sd_y = math.sqrt(var_y) / scale

    out = []
    for elem in t:
        out.append([sd_x*np.cos(elem) , sd_y*np.sin(elem)])
    
    out_rot = []

    for elem in out:
        out_rot.append([elem[0] * np.cos(-th) + elem[1] * np.sin(-th), elem[1] * np.cos(-th) - elem[0] * np.sin(-th)])

    out_shift = []
    for elem in out_rot:
        out_shift.append([elem[0]  + x, elem[1] + y])
        #elem[0] = elem[0] + x
        #elem[1] = elem[1] + y

    #return out
    #return out_rot
    return out_shift

    

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    #parser.add_argument('--title', nargs=1, required=True)
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    plot_errors(filenames[0])
