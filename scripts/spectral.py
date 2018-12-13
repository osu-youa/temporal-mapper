import numpy as np
import numpy.fft as fft
import pdb

def shorten_data_representation(array, num_freqs = 3):

    assert np.all(array == np.conj(array)), 'This function only works on real-valued data inputs'

    n = len(array)
    num_freqs = min(num_freqs, (n + 1) // 2 - 1)
    transform = fft.fft(array)

    zero_freq = transform[0]
    aliased_freq = None
    if n % 2 == 0:
        if num_freqs == (n + 1) // 2 - 1:
            aliased_freq = transform[n/2]
        else:
            aliased_freq = 0

    pos_freqs = transform[1:n//2]
    abs_freqs = np.abs(pos_freqs)

    # Grab the index positions of the n-1 largest frequency values by absolute value
    idx = np.argpartition(abs_freqs, -(num_freqs))[-(num_freqs):]
    vals = pos_freqs[idx]

    rez = {
        'n': n,
        'zero_freq': zero_freq,
        'aliased_freq': aliased_freq,
        'index': idx,
        'values': vals,
    }

    return rez

def expand_data_representation(data_dict):
    """
    Takes the output of shorten_data_representation and transforms it back into the corresponding DFT representation
    :param data_dict: A dictionary formatted as in shorten_data_representation.
    :return: A numpy array representing the transformed values
    """

    n = data_dict['n']

    transform = np.zeros(n, dtype=np.complex)
    transform[0] = data_dict['zero_freq']
    if n % 2 == 0:
        transform[n/2] = data_dict['aliased_freq']

    pos_freqs = np.zeros((n+1)//2 - 1, dtype=np.complex)
    pos_freqs[data_dict['index']] = data_dict['values']
    neg_freqs = np.conj(pos_freqs[::-1])

    transform[1:len(pos_freqs)+1] = pos_freqs
    transform[-len(pos_freqs):] = neg_freqs

    rez = fft.ifft(transform)

    return rez

def get_spectral_decomposition(array, num_freqs=3):
    return expand_data_representation(shorten_data_representation(array, num_freqs=num_freqs))

if __name__ == '__main__':
    test_vals = np.sin(np.pi*np.linspace(0, 5, 50))

    import matplotlib.pyplot as plt

    # Try full rep

    # Bug here with 25, full matrix is not reconstructing! Figure out why

    rez = shorten_data_representation(test_vals, 3)
    new_vals = expand_data_representation(rez)

    plt.plot(test_vals)
    plt.plot(new_vals)
    plt.show()



