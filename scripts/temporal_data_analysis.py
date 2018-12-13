import matplotlib.pyplot as plt
import cPickle
import pdb
import numpy as np


file_name = '/home/prg/temporal_data.pickle'

def run_analysis():
    with open(file_name, 'rb') as fh:
        data = cPickle.load(fh)



    for point in data:
        mean_val = np.mean(data[point].values())
        plt.plot([point[0]], [point[1]], marker='s', color=str(mean_val))

    plt.show()

    pdb.set_trace()