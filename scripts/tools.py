import matplotlib.pyplot as plt
import cPickle
import getpass
import pdb
import numpy as np
import stats_autocorr
from temporal_map_publisher import INTERPOLATION_INTERVAL

data = {}

def get_data(debug=False):
    if not data:
        with open('/home/{}/temporal_data_idle.pickle'.format(getpass.getuser()), 'rb') as fh:
            file_contents = cPickle.load(fh)
        global data
        data = file_contents

    return data

def check_point(x,y, plot=True, debug=False):
    point = (x,y)
    data = get_data()
    ts = data.get(point)
    if not ts:
        print('No data associated with {}'.format(point))
        return

    ts_data = np.array(list(ts.iteritems()))
    sort_index = np.argsort(ts_data[:, 0])
    ts_data = ts_data[sort_index]



    interpolation = stats_autocorr.linearly_interpolate_data(ts_data[:, 0], ts_data[:, 1], INTERPOLATION_INTERVAL)
    steps = np.arange(ts_data[0,0], ts_data[-1,0], INTERPOLATION_INTERVAL)

    periods_returned, autocorrs_returned = stats_autocorr.get_autocorrelations(interpolation)
    periods_all, autocorrs_all = stats_autocorr.get_autocorrelations(interpolation, p_value_filter=1.0, filter_negative=False, use_local_maxima=False)


    if plot:
        plt.subplot(2,1,1)
        plt.plot(steps, interpolation)
        plt.scatter(ts_data[:,0], ts_data[:,1])
        plt.title('Observations')
        plt.xlabel('Time')
        plt.ylabel('Occupancy')

        plt.subplot(2,1,2)
        plt.plot(periods_all * INTERPOLATION_INTERVAL, autocorrs_all)
        plt.scatter(periods_returned * INTERPOLATION_INTERVAL, autocorrs_returned)
        plt.title('Computed Autocorrelations')
        plt.ylabel('Autocorrelation')
        plt.xlabel('Period')

        plt.show()

    if debug:
        pdb.set_trace()

    return interpolation