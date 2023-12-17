import matplotlib.pyplot as plt
from utilities import FileReader




def plot_errors(angular_file, linear_file):
    
    

    
    
    fig, axes = plt.subplots(1,2, figsize=(14,6))

    

    headers, values=FileReader(angular_file).read_file()
    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    axes[0].set_title("angular")
    for i in range(0, len(headers) - 1):
        axes[0].plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")

    axes[0].legend()
    axes[0].grid()
    axes[0].set_xlabel("Time (ns)")

    headers, values=FileReader(linear_file).read_file()
    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    
    axes[1].set_title("linear")
    for i in range(0, len(headers) - 1):
        axes[1].plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")

    axes[1].legend()
    axes[1].grid()
    axes[1].set_xlabel("Time (ns)")
    plt.show()

import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    #parser.add_argument('--title', nargs=1, required=True)
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    plot_errors(filenames[0], filenames[1])