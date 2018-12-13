import numpy as np
import scipy.stats as stats
from scipy.signal import argrelmax
import spectral

def linearly_interpolate_data(orig_t, orig_val, step_size):

    assert all(np.diff(orig_t) > 0)
    steps = np.arange(orig_t[0], orig_t[-1], step_size)
    return np.interp(steps, orig_t, orig_val)

def get_autocorrelations(data, limit_steps_max = None, limit_steps = None, p_value_filter=0.05,
                         filter_negative=True, use_local_maxima=True):
    """
    Takes a series of data computes autocorrelations in the data up until len(data)/2, unless different limits are
    specified in which case those limits will be used instead.

    :param data: A series of linearly interpolated data, such as produced by linearly_interpolate_data
    :param limit_steps_max: If specified, only autocorrelation cycles up until the given integer value will be produced.
    :param limit_steps: If specified, only the provided list of integer values will be tested for autocorrelation.
    :param p_value_filter: Only returns entries below the specified threshold. Set to 1 to return everything.
    :param filter_negative: Only returns positive autocorrelations if set to True.
    :return: A list of autocorrelations as well as the corresponding cycles.
    """
    if limit_steps is not None:
        periods = limit_steps

    else:
        periods = xrange(1, min(len(data) // 2, np.inf if limit_steps_max is None else limit_steps_max))

    n = len(data)
    info = []

    for period in periods:

        autocorr, p_val = stats.pearsonr(data[:n-period], data[period:])
        info.append((period, autocorr, p_val))

    info = np.array(info)

    indicator = np.ones(len(info), dtype=bool)

    # P-val filter
    indicator &= info[:,2] <= p_value_filter

    # If filter_negative is on, filter out all negative autocorrs
    if filter_negative:
        indicator &= info[:,1] > 0

    # Filter out anything that's not a local maximum for autocorrs
    if use_local_maxima:
        maxima_arguments = argrelmax(info[:,1])
        max_indicator = np.zeros(len(info), dtype=bool)
        max_indicator[maxima_arguments] = True
        indicator &= max_indicator

    info = info[indicator]

    # Return the periods of interest and the corresponding autocorrelations
    return info[:,0].copy(), info[:,1].copy()


def get_autocorrelations_spectral(data, freqs=3, p_value_filter=0.05):
    n = len(data)

    candidate_frequencies = spectral.shorten_data_representation(data, num_freqs=freqs)['index'] + 1.0
    candidate_periods = (np.round(n / candidate_frequencies)).astype(np.int)

    rez = []

    for period in candidate_periods:
        autocorr, p_val = stats.pearsonr(data[:-period], data[period:])
        if p_val <= p_value_filter and autocorr > 0:
            rez.append((period, autocorr))

    if not rez:
        return np.array([]), np.array([])

    rez = np.array(rez)

    return rez[:,0].copy(), rez[:,1].copy()


