

import matplotlib.pyplot as plt
from utilities import FileReader




def plot_errors(filename): 

    headers, values=FileReader(filename).read_file()
    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

 
    plt.plot([lin[4] for lin in values], [lin[5] for lin in values], label= "enc")
    plt.plot([lin[11] for lin in values], [lin[12] for lin in values], label= "kf")
    plt.title("state space")
    plt.grid()
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.legend()

    plt.show()

import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    #parser.add_argument('--title', nargs=1, required=True)
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    plot_errors(filenames[0])
