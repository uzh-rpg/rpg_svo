#!/usr/bin/python

import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc

rc('font',**{'family':'serif','serif':['Cardo']})
rc('text', usetex=True)

def precision_plot(ax, errors, max_error, color, linestyle):
    number_of_error_ranges = 500
    error_step = max_error / number_of_error_ranges
    precision_array = np.zeros(number_of_error_ranges)
    error_array = np.zeros(number_of_error_ranges)
    n_measurements = len(errors)
    for error_ind in range(0, number_of_error_ranges):
        precision_array[error_ind] = np.sum(errors < error_step*error_ind) / float(n_measurements) * 100.0
        error_array[error_ind] = error_step*error_ind
    plot_obj, = ax.plot(error_array, precision_array, label='error', linestyle=linestyle, color=color)
    return plot_obj
    
def analyse_depth(results_dir):
    D = np.loadtxt(os.path.join(results_dir, 'depth_error.txt'), delimiter=' ')
    D = D[D[:,0] > 1,:]

    fig = plt.figure(figsize=(6,5))
    ax = fig.add_subplot(111, xlabel='depth error [m]', ylabel='Precision [\%]')
    errors = np.abs(D[:,1])
    precision_plot(ax, errors, 0.05, 'r', '-')    
    fig.tight_layout()
    fig.savefig(os.path.join(results_dir,'depth_error.pdf'), bbox_inches="tight")
    

def plot_depth_over_time(results_dir, ax, x_axis_data, color, label=''):
    D = np.loadtxt(os.path.join(results_dir, 'depth_error.txt'), delimiter=' ')
    D = D[D[:,0] > 1,:]
    idxs = np.unique(D[:,0])
    percentile_10 = np.zeros(len(idxs))
    percentile_50 = np.zeros(len(idxs))
    percentile_90 = np.zeros(len(idxs))
    for i in range(len(idxs)):
        errors = np.abs(D[D[:,0]==idxs[i],1])
        percentile_10[i] = np.percentile(errors, 10)
        percentile_50[i] = np.percentile(errors, 50)
        percentile_90[i] = np.percentile(errors, 90)
    if len(x_axis_data) == 0:
        x_axis_data = idxs
    print np.shape(x_axis_data)
    print np.shape(idxs)
    print '--'
    ax.plot(x_axis_data, percentile_50, linewidth=2, color=color, label=label)
    ax.plot(x_axis_data, percentile_10, linewidth=0.5, color=color, alpha=0.5)
    ax.plot(x_axis_data, percentile_90, linewidth=0.5, color=color, alpha=0.5)
    ax.fill_between(x_axis_data, percentile_10, percentile_90, color=color, alpha=0.1)
     
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Analyse depth estimate')
    parser.add_argument('results_dir', help='folder with the results')
    args = parser.parse_args()
    analyse_depth(args.results_dir)
    
    fig = plt.figure(figsize=(6,5))
    ax = fig.add_subplot(111, xlabel='Measurement', ylabel='Scale-Drift')
    plot_depth_over_time(args.results_dir, [], ax)