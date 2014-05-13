#!/usr/bin/python

import os
import sys
import time
import rospkg
import numpy as np
import matplotlib.pyplot as plt
import yaml
import argparse
from matplotlib import rc

# tell matplotlib to use latex font
rc('font',**{'family':'serif','serif':['Cardo']})
rc('text', usetex=True)    
  
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
from mpl_toolkits.axes_grid1.inset_locator import mark_inset

def plot_trajectory(ax, filename, label, color, linewidth):
    file = open(filename)
    data = file.read()
    lines = data.replace(","," ").replace("\t"," ").split("\n")
    trajectory = np.array([[v.strip() for v in line.split(" ") if v.strip()!=""] for line in lines if len(line)>0 and line[0]!="#"], dtype=np.float64)
    ax.plot(trajectory[:,1], trajectory[:,2], label=label, color=color, linewidth=linewidth)

def save_figure(fig, name, directory):
    fig.tight_layout()
    fig.savefig(os.path.join(directory, name+'.pdf'), bbox_inches="tight")

def distances_along_trajectory(traj):
    keys = traj.keys()
    keys.sort()
    motion = [ominus(traj[keys[i+1]],traj[keys[i]]) for i in range(len(keys)-1)]
    distances = [0]
    sum = 0
    for t in motion:
        sum += compute_distance(t)
        distances.append(sum)
    return distances
    
def compare_results(comp_params, results_dir, comparison_dir):
    print('run comparison: '+comp_params['comparison_name'])
           
    
    # -------------------------------------------------------------------------
    # plot trajectory:
    fig = plt.figure(figsize=(6,2))
    ax = fig.add_subplot(111, xlabel='x [m]', ylabel='y [m]')
    for exp_set in comp_params['experiment_sets']:
        print('processing experiment set: ' + exp_set['label'])
        
        for i, exp in enumerate(exp_set['experiments']):
            data = np.loadtxt(os.path.join(results_dir, exp, 'traj_estimate.txt'))
            if i == 0:
                base_plot, = ax.plot(data[:,1], data[:,2], label=exp_set['label'])
            else:
                ax.plot(data[:,1], data[:,2], color=base_plot.get_color())
    ax.legend(loc='upper left')
    save_figure(fig, 'trajectory', comparison_dir)
    
    
    # -------------------------------------------------------------------------
    # plot translation error:
    fig = plt.figure(figsize=(6,2))
    ax = fig.add_subplot(111, xlabel='distance [m]', ylabel='translation drift [mm]')
    for exp_set in comp_params['experiment_sets']:
        print('processing experiment set: ' + exp_set['label'])
        
        for i, exp in enumerate(exp_set['experiments']):
            gt = np.loadtxt(os.path.join(results_dir, exp, 'groundtruth_matched.txt'))
            distances = np.diff(gt[:,1:4],axis=0)
            distances = np.sqrt(np.sum(np.multiply(distances,distances),1))
            distances = np.cumsum(distances)
            distances = np.concatenate(([0], distances))
            data = np.loadtxt(os.path.join(results_dir, exp, 'translation_error.txt'))
            e = np.sqrt(np.sum(np.multiply(data[:,1:4],data[:,1:4]),1))
            
            if np.shape(e)[0] > np.shape(distances)[0]:
                print('WARNING: estimate has more measurement than groundtruth: '
                      +str(np.shape(e)[0]-np.shape(distances)[0]))
                e = e[0:np.shape(distances)[0]]
                
            distances = distances[0:np.shape(e)[0]+1]
           
            
            if i == 0:
                base_plot, = ax.plot(distances, e*1000, label=exp_set['label'])
            else:
                ax.plot(distances, e*1000, color=base_plot.get_color())
                
    ax.legend(loc='upper left')
    save_figure(fig, 'translation_error', comparison_dir)
  
if __name__ == '__main__':
  
    parser = argparse.ArgumentParser(description='Compare results of a VO pipeline.')
    parser.add_argument('comparison_file', help='A YAML file that contains the details of the comparison')
    args = parser.parse_args()
     
    # load comparison file
    args.experiment_file = args.comparison_file.replace('.yaml','')
    comparison_params_file = os.path.join(rospkg.RosPack().get_path('svo_analysis'), 
                                          'comparisons', args.comparison_file+'.yaml')
    if os.path.exists(comparison_params_file):
        comp_params = yaml.load(open(comparison_params_file, 'r'))
    else:
        raise Exception("Provided comparison file does not exist.")
        
    # create folder for comparison results
    comparison_dir = os.path.join(rospkg.RosPack().get_path('svo_analysis'), 
                                  'comparisons', comp_params['comparison_name'])
    if not os.path.exists(comparison_dir):
        os.makedirs(comparison_dir)

    # folder where the results of previous experiments are saved    
    results_dir = os.path.join(rospkg.RosPack().get_path('svo_analysis'), 'results')
    
    # run comparison
    compare_results(comp_params, results_dir, comparison_dir) 
                
           
