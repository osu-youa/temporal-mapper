import spectral
import numpy as np
import pdb
import cPickle
import matplotlib.pyplot as plt
import stats_autocorr
import tools

def visualize_data_sets(data_type='step', freq=3):

    if data_type == 'step':
        data_set = np.array([0.] * 20 + [1.] * 20)
    elif data_type == 'large step':
        data_set = np.array([0.0] * 1 + [1.] * 39)
    elif data_type == 'periodic even':
        data_set = np.array([0.] * 10 + [1.0] * 10 + [0.0] * 10 + [1.0] * 10)
    elif data_type == 'periodic uneven':
        data_set = np.array([0.] * 17 + [1.0] * 3 + [0.0] * 17 + [1.0] * 3)
    elif data_type == '2 period':

        f_1 = [1.0] * 4 + [0.0] * 4
        a_1 = np.array(f_1 * 5)

        f_2 = [0.0] * 3 + [1.0] * 2
        a_2 = np.array(f_2 * 8)

        data_set = (a_1 + a_2)/2
    elif data_type == 'periodic prime':
        data_set = np.array([1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0] * 6)[:40]
    elif data_type == 'uniform':
        data_set = np.array([1.0] * 40)
    elif isinstance(data_type, tuple):
        kwargs = {'plot': False, 'debug': False}
        data_set = tools.check_point(*data_type, **kwargs)
    else:
        raise ValueError('Unknown data_type {}'.format(data_type))

    all_freqs = spectral.shorten_data_representation(data_set, num_freqs=len(data_set))
    shortened_freqs = spectral.shorten_data_representation(data_set, num_freqs=freq)
    recon = spectral.expand_data_representation(shortened_freqs)

    # Get the array indexes which are not the most frequent ones
    all_indexes = all_freqs['index']

    periods, autocorrs = stats_autocorr.get_autocorrelations_spectral(data_set)
    if periods is not None:
        print('Periods detected! {}'.format(', '.join([str(int(x)) for x in periods])))
    else:
        print('No periods detected.')

    plt.subplot(2, 1, 1)
    plt.plot(data_set, label='Original')
    plt.plot(np.real(recon), label='Reconstructed')
    plt.ylabel('Occupancy')

    plt.subplot(2, 1, 2)
    plt.bar(all_indexes + 1, np.abs(all_freqs['values']), color='blue')
    # plt.bar(top_index + 1, np.abs(shortened_freqs['values']), color='red')
    plt.title('Frequencies')
    plt.ylabel('Magnitude')

    plt.show()
    pdb.set_trace()


def test_clustering_algorithm(variance=1.0, samples=50, debug=False):

    default_colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'pink']

    import hdbscan

    means = [(1,1), (5,5), (3,0)]
    data_samples = []
    for mean in means:
        data_samples.append(np.random.multivariate_normal(mean, np.diag(np.ones(len(mean))*variance), samples))

    noise = []
    for i in range(100):
        noise.append(np.random.random(len(means[0]))*20-10)
    data_samples.append(np.array(noise))

    data = np.concatenate(data_samples)
    categories = hdbscan.HDBSCAN(min_cluster_size=10).fit_predict(data)

    for i, category in enumerate(np.unique(categories)):
        ind = categories == category

        plt.scatter(data[ind, 0], data[ind, 1], color=default_colors[i % len(default_colors)])

    plt.show()

    if debug:
        pdb.set_trace()


