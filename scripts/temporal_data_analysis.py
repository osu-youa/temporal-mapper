import numpy as np
import pandas as pd
import getpass
import pdb
import os
import cPickle
from pandas import Series, DataFrame
from pandas import datetools as dt
import matplotlib.pyplot as plt
import spectral

DATA_DIR = '/home/{}/data_files'.format(getpass.getuser())
FILES = ['ts_1.pickle',
        # 'ts_2.pickle',
         ]
TIMEZONE = 'America/Los_Angeles'

NORMALIZATION_PARAMS = {
    'm': ('minute', 60),
    's': ('second', 1),
    'h': ('hour', 3600),
}

def convert_timestamp_index(obj, tz = TIMEZONE):
    raw = obj.index
    dates = dt.to_datetime(raw, unit='s').tz_localize('UTC').tz_convert(tz)
    return obj.rename(index=dict(zip(raw, dates)))

def get_data():
    rez = {}
    for file_name in FILES:
        path = os.path.join(DATA_DIR, file_name)
        with open(path, 'rb') as fh:
            rez.update(cPickle.load(fh))

    ts = Series(rez)
    return convert_timestamp_index(ts)


def get_rolling_data(window=60, step=5, normalization_unit='m'):
    """
    Takes the entry/exit data
    :param window:
    :param step:
    :param normalization_unit:
    :return:
    """

    if window % step:
        raise ValueError('The window argument should be a multiple of the step argument')

    ts = get_data()
    df = DataFrame(index=ts.index)
    df['entry'] = ts.astype(int)
    df['exit'] = (~ts).astype(int)
    df['timestamp'] = df.index.astype(np.int64)/1.e9

    floored_timestamp = (df['timestamp'] - (df['timestamp'] % step)).astype(np.int64)

    activity = df[['entry', 'exit']].groupby(floored_timestamp).sum()
    slots = np.arange(activity.index.min(), activity.index.max() + step, step)
    activity = activity.reindex(slots).fillna(0)

    window_num_obs = window / step

    rolling_activity = convert_timestamp_index(pd.rolling_sum(activity, window_num_obs, min_periods=1))

    normalization_constant = NORMALIZATION_PARAMS[normalization_unit][1]
    rolling_activity *= normalization_constant / float(window)

    return rolling_activity



