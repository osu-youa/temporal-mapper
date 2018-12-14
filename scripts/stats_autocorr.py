import numpy as np
import scipy.stats as stats
from scipy.signal import argrelmax
import spectral
import pdb

def linearly_interpolate_data(orig_t, orig_val, step_size):

    assert all(np.diff(orig_t) > 0)
    if len(orig_t) == 1:
        return orig_val, orig_t, np.array([True])

    # This part just creates the interpolated data
    steps = np.arange(orig_t[0], orig_t[-1], step_size)
    interpolated_rez = np.interp(steps, orig_t, orig_val)

    """
    This part determines which interpolations are "close" enough to an existing point that they can be considered
    valid interpolations.

    First, we take the original data samples and "expand" them to create contiguous intervals.
    We then iterate through the interpolated time values and ensure that they fall within these contiguous
    intervals.
    
    E.g. suppose our original time sample data is
    ||.......||||..||...|||
    
    We expand these to form contiguous intervals
    [ ].....[    ][  ].[  ]
    
    Then the interpolated values we keep are:
    |||.....||||||||||.|||| 
    """

    # Interval construction
    intervals = []
    current_interval = None
    last_timestamp = None

    for index, timestamp in enumerate(orig_t):
        if index != len(orig_t) - 1:

            # Case 1:
            if current_interval is None:
                current_interval = [timestamp - step_size, None]
                last_timestamp = timestamp
                continue

            # Case 2: Our original t-values are close enough to be considered contiguous
            if last_timestamp + step_size > timestamp - step_size:
                last_timestamp = timestamp

            # Case 3: Our t-value isn't close enough to the previous one, so we close off the previous interval
            # and start a new one
            else:
                current_interval[1] = last_timestamp + step_size
                intervals.append(tuple(current_interval))

                current_interval = [timestamp - step_size, None]
                last_timestamp = timestamp

    # Close off the final interval
    current_interval[1] = last_timestamp + step_size
    intervals.append(tuple(current_interval))


    # Interval comparison
    mask = np.zeros(len(steps), dtype=np.bool)
    for start, end in intervals:
        mask |= (start < steps) & (steps < end)

    return interpolated_rez, steps, mask

def get_autocorrelations(data, limit_steps_max = None, limit_steps = None, p_value_filter=0.05,
                         filter_negative=True, use_local_maxima=True, data_mask=None):
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

    if data_mask is not None:
        data[~data_mask] = np.nan

    for period in periods:

        orig_data = data[:-period]
        shift_data = data[period:]

        data_to_keep = ~np.isnan(orig_data) & ~np.isnan(shift_data)
        if data_to_keep.sum() < 10:
            info.append((period, np.nan, np.nan))
            continue


        autocorr, p_val = stats.pearsonr(orig_data[data_to_keep], shift_data[data_to_keep])
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


def get_autocorrelations_spectral(data, freqs=3, p_value_filter=0.05, data_mask=None):
    n = len(data)

    candidate_frequencies = spectral.shorten_data_representation(data, num_freqs=freqs)['index'] + 1.0
    candidate_periods = (np.round(n / candidate_frequencies)).astype(np.int)

    rez = []

    if data_mask is not None:
        data[~data_mask] = np.nan

    for period in candidate_periods:

        orig_data = data[:-period]
        shift_data = data[period:]

        data_to_keep = ~np.isnan(orig_data) & ~np.isnan(shift_data)
        if data_to_keep.sum() < 10:
            continue

        autocorr, p_val = stats.pearsonr(orig_data[data_to_keep], shift_data[data_to_keep])

        if p_val <= p_value_filter and autocorr > 0:
            rez.append((period, autocorr))

    if not rez:
        return np.array([]), np.array([])

    rez = np.array(rez)

    return rez[:,0].copy(), rez[:,1].copy()
