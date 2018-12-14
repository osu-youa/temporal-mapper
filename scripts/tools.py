import matplotlib.pyplot as plt
import cPickle
import getpass
import pdb
import numpy as np
import stats_autocorr
from temporal_map_publisher import INTERPOLATION_INTERVAL
import spectral

data = {}

def get_data(debug=False):
    if not data:
        with open('/home/{}/temporal_data_20181213.pickle'.format(getpass.getuser()), 'rb') as fh:
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

    interpolation, steps, mask = stats_autocorr.linearly_interpolate_data(ts_data[:, 0], ts_data[:, 1], INTERPOLATION_INTERVAL)

    freq_data = spectral.shorten_data_representation(interpolation, num_freqs=3)
    candidate_periods = np.round(freq_data['n'] / (freq_data['index'] + 1.0)).astype(np.int)

    decomp = spectral.expand_data_representation(freq_data)


    # periods_returned, autocorrs_returned = stats_autocorr.get_autocorrelations(interpolation, data_mask=mask)
    periods_all, autocorrs_all = stats_autocorr.get_autocorrelations(interpolation, p_value_filter=1.0,
                                                                     filter_negative=False, use_local_maxima=False,
                                                                     data_mask=mask)
    # Hack: Sometimes the candidate period is the length of the data, in which case
    # there's no autocorrelation

    candidate_autocorrs = []
    candidate_periods = candidate_periods[candidate_periods < len(interpolation)]
    for period in candidate_periods:
        try:
            index = np.where(periods_all == period)[0][0]
            candidate_autocorrs.append(autocorrs_all[index])
        except IndexError:
            candidate_autocorrs.append(np.nan)

    candidate_autocorrs = np.array(candidate_autocorrs)



    interpolation[~mask] = np.nan

    if plot:
        plt.subplot(2,1,1)
        plt.plot(steps, interpolation)
        plt.plot(steps, decomp, linestyle=':', color='green' )
        plt.title('Observations ({}, {})'.format(x,y))
        plt.xlabel('Time')
        plt.ylabel('Occupancy')

        plt.subplot(2,1,2)
        plt.plot(periods_all * INTERPOLATION_INTERVAL, autocorrs_all)
        plt.scatter(candidate_periods * INTERPOLATION_INTERVAL, candidate_autocorrs)
        plt.title('Computed Autocorrelations')
        plt.ylabel('Autocorrelation')
        plt.xlabel('Period')

        plt.show()

    if debug:
        pdb.set_trace()

    return interpolation