import matplotlib.pyplot as plt
from utilities import FileReader
import math


def plot_errors(filename):
    
    headers, values=FileReader(filename).read_file()
    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    
    
    fig, axes = plt.subplots(2,1, figsize=(14,6))

    axes[0].set_title("a_x Comparison")
    axes[0].plot(time_list, [lin[0] for lin in values], label= headers[0])
    axes[0].plot(time_list, [lin[7] for lin in values], label= headers[7])
    axes[0].legend()
    axes[0].grid()
    
    axes[1].set_title("a_y Comparison")
    axes[1].plot(time_list, [lin[1] for lin in values], label= headers[1])
    axes[1].plot(time_list, [lin[8] for lin in values], label= headers[8])
    axes[1].legend()
    axes[1].grid()

    plt.show()

    fig, axes = plt.subplots(2,1, figsize=(14,6))

    axes[0].set_title("vx Comparison")
    axes[0].plot(time_list, [lin[2] for lin in values], label= headers[2])
    axes[0].plot(time_list, [lin[9] for lin in values], label= headers[9])
    axes[0].legend()
    axes[0].grid()
    
    axes[1].set_title("w Comparison")
    axes[1].plot(time_list, [lin[3] for lin in values], label= headers[3])
    axes[1].plot(time_list, [lin[10] for lin in values], label= headers[10])
    axes[1].legend()
    axes[1].grid()

    plt.show()

    fig, axes = plt.subplots(3,1, figsize=(14,4))

    axes[0].set_title("x Comparison")
    axes[0].plot(time_list, [lin[4] for lin in values], label= headers[4])
    axes[0].plot(time_list, [lin[11] for lin in values], label= headers[11])
    axes[0].legend()
    axes[0].grid()
    
    axes[1].set_title("y Comparison")
    axes[1].plot(time_list, [lin[5] for lin in values], label= headers[5])
    axes[1].plot(time_list, [lin[12] for lin in values], label= headers[12])
    axes[1].legend()
    axes[1].grid()

    axes[2].set_title("th Comparison")
    axes[2].plot(time_list, [math.asin(math.sin(lin[6])) for lin in values], label= headers[6])
    axes[2].plot(time_list, [math.asin(math.sin(lin[13])) for lin in values], label= headers[13])
    axes[2].legend()
    axes[2].grid()

    plt.show()

import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    #parser.add_argument('--title', nargs=1, required=True)
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    for filename in filenames:
        plot_errors(filename)