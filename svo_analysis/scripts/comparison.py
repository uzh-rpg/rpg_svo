#!/usr/bin/python

import os
import sys
import time
import rospkg
import numpy as np
import matplotlib.pyplot as plt
import yaml
import argparse
import svo_analysis.analyse_depth as analyse_depth
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
    
def get_distance_from_start(gt):
    distances = np.diff(gt[:,1:4],axis=0)
    distances = np.sqrt(np.sum(np.multiply(distances,distances),1))
    distances = np.cumsum(distances)
    distances = np.concatenate(([0], distances))
    return distances
    
def compare_results(comp_params, results_dir, comparison_dir):
    print('run comparison: '+comp_params['comparison_name'])
           
    line_styles = ['-','--',':']
    line_colors = ['b','g','r','m','c']
    
    # -------------------------------------------------------------------------
    # plot trajectory:
    fig = plt.figure(figsize=(6,2))
    ax = fig.add_subplot(111, xlabel='x [m]', ylabel='y [m]')
    for exp_set in comp_params['experiment_sets']:
        print('processing experiment set: ' + exp_set['label'])
        
        for i, exp in enumerate(exp_set['experiments']):
            data = np.loadtxt(os.path.join(results_dir, exp, 'traj_estimate.txt'))
            if i == 0:
                base_plot, = ax.plot(data[:,1], data[:,2], label=exp_set['label'], linestyle=line_styles[np.mod(i, len(line_styles))])
            else:
                ax.plot(data[:,1], data[:,2], color=base_plot.get_color(), linestyle= line_styles[np.mod(i, len(line_styles))])
    ax.legend(loc='upper left', ncol=3)
    save_figure(fig, 'trajectory', comparison_dir)
    
    # -------------------------------------------------------------------------
    # plot translation error:
    fig = plt.figure(figsize=(6,2))
    ax = fig.add_subplot(111, xlabel='distance [m]', ylabel='translation drift [mm]')
    for exp_set in comp_params['experiment_sets']:
        print('processing experiment set: ' + exp_set['label'])
        
        for i, exp in enumerate(exp_set['experiments']):
            gt = np.loadtxt(os.path.join(results_dir, exp, 'groundtruth_matched.txt'))
            distances = get_distance_from_start(gt)
            data = np.loadtxt(os.path.join(results_dir, exp, 'translation_error.txt'))
            e = np.sqrt(np.sum(np.multiply(data[:,1:4],data[:,1:4]),1))
            
            if np.shape(e)[0] > np.shape(distances)[0]:
                print('WARNING: estimate has more measurement than groundtruth: '
                      +str(np.shape(e)[0]-np.shape(distances)[0]))
                e = e[0:np.shape(distances)[0]]
                
            distances = distances[0:np.shape(e)[0]]
            
            print '--'
            print np.shape(e)
            print np.shape(distances)
            
            if i == 0:
                base_plot, = ax.plot(distances, e*1000, label=exp_set['label'], linestyle= line_styles[np.mod(i, len(line_styles))])
            else:
                ax.plot(distances, e*1000, color=base_plot.get_color(), linestyle= line_styles[np.mod(i, len(line_styles))])
                
    ax.legend(loc='upper left', ncol=3)
    save_figure(fig, 'translation_error', comparison_dir)
    
    # -------------------------------------------------------------------------
    # plot depth estimation error:
    fig = plt.figure(figsize=(6,5))
    ax = fig.add_subplot(111, xlabel='Travelled distance [m]', ylabel='Error [m]')
    for k, exp_set in enumerate(comp_params['experiment_sets']):
        print('plot depth error for experiment: ' + exp)
        exp = exp_set['experiments'][0]
        gt = np.loadtxt(os.path.join(results_dir, exp, 'groundtruth_matched.txt'))
        x_axis_data = get_distance_from_start(gt)
        analyse_depth.plot_depth_over_time(os.path.join(results_dir, exp), ax,
                                           x_axis_data[1:], line_colors[k], exp_set['label'])
        ax.legend(loc='upper left', ncol=3)
    save_figure(fig, 'depth_error_textures', comparison_dir)
    
    fig = plt.figure(figsize=(6,5))
    ax = fig.add_subplot(111, xlabel='Travelled distance [m]', ylabel='Error [m]')
    exp_set = comp_params['experiment_sets'][0]
    for i, exp in enumerate(exp_set['experiments']):
        print('plot depth error for speed: ' + exp)
        gt = np.loadtxt(os.path.join(results_dir, exp, 'groundtruth_matched.txt'))
        x_axis_data = get_distance_from_start(gt)
        params = yaml.load(open(os.path.join(results_dir, exp, 'dataset_params.yaml')))
        analyse_depth.plot_depth_over_time(os.path.join(results_dir, exp), ax,
                                           x_axis_data[1:], line_colors[i],
                                           str(params['trajectory_modifiers']['speed'])+' m/s')
    ax.legend(loc='upper left', ncol=3)
    save_figure(fig, 'depth_error_speed', comparison_dir)
  
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
                
           
