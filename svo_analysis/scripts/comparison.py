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

def compare_results(experiments, results_dir, comparison_dir, 
                    plot_scale_drift = False):
  
  # ------------------------------------------------------------------------------
  # position error 
  fig_poserr = plt.figure(figsize=(8,6))
  ax_poserr_x = fig_poserr.add_subplot(311, ylabel='x-error [m]')
  ax_poserr_y = fig_poserr.add_subplot(312, ylabel='y-error [m]')
  ax_poserr_z = fig_poserr.add_subplot(313, ylabel='z-error [m]', xlabel='time [s]')
  
  for exp in experiments:
    
    # load dataset parameters
    params_stream = open(os.path.join(results_dir, exp, 'params.yaml'))
    params = yaml.load(params_stream)
  
    # plot translation error
    trans_error = np.loadtxt(os.path.join(results_dir, exp, 'translation_error.txt'))
    trans_error[:,0] = trans_error[:,0]-trans_error[0,0]
    ax_poserr_x.plot(trans_error[:,0], trans_error[:,1], label=params['experiment_label'])
    ax_poserr_y.plot(trans_error[:,0], trans_error[:,2])
    ax_poserr_z.plot(trans_error[:,0], trans_error[:,3])
    
  ax_poserr_x.set_xlim([0, trans_error[-1,0]+4])
  ax_poserr_y.set_xlim([0, trans_error[-1,0]+4]) 
  ax_poserr_z.set_xlim([0, trans_error[-1,0]+4])
  ax_poserr_x.legend(bbox_to_anchor=[0, 0], loc='lower left', ncol=3)
  ax_poserr_x.grid()
  ax_poserr_y.grid()
  ax_poserr_z.grid()
  fig_poserr.tight_layout()
  fig_poserr.savefig(os.path.join(comparison_dir, 'translation_error.pdf'))
  
  # ------------------------------------------------------------------------------
  # orientation error 
  fig_roterr = plt.figure(figsize=(8,6))
  ax_roterr_r = fig_roterr.add_subplot(311, ylabel='roll-error [rad]')
  ax_roterr_p = fig_roterr.add_subplot(312, ylabel='pitch-error [rad]')
  ax_roterr_y = fig_roterr.add_subplot(313, ylabel='yaw-error [rad]', xlabel='time [s]')
  
  for exp in experiments:
    
    # load dataset parameters
    params_stream = open(os.path.join(results_dir, exp, 'params.yaml'))
    params = yaml.load(params_stream)
  
    # plot translation error
    rot_error = np.loadtxt(os.path.join(results_dir, exp, 'orientation_error.txt'))
    rot_error[:,0] = rot_error[:,0]-rot_error[0,0]
    ax_roterr_r.plot(rot_error[:,0], rot_error[:,3], label=params['experiment_label'])
    ax_roterr_p.plot(rot_error[:,0], rot_error[:,2])
    ax_roterr_y.plot(rot_error[:,0], rot_error[:,1])
    
  ax_roterr_r.set_xlim([0, rot_error[-1,0]+4])
  ax_roterr_p.set_xlim([0, rot_error[-1,0]+4])
  ax_roterr_y.set_xlim([0, rot_error[-1,0]+4])
  ax_roterr_r.legend(bbox_to_anchor=[0, 1], loc='upper left', ncol=3)
  ax_roterr_r.grid()
  ax_roterr_p.grid()
  ax_roterr_y.grid()
  fig_roterr.tight_layout()
  fig_roterr.savefig(os.path.join(comparison_dir, 'orientation_error.pdf'))
  
  # ------------------------------------------------------------------------------
  # scale error 
  if plot_scale_drift:
    fig_scale = plt.figure(figsize=(8,2.5))
    ax_scale = fig_scale.add_subplot(111, xlabel='time [s]', ylabel='scale change [\%]')
    
    for exp in experiments:
      
      # load dataset parameters
      params = yaml.load(open(os.path.join(results_dir, exp, 'params.yaml')))
    
      # plot translation error
      scale_drift = open(os.path.join(results_dir, exp, 'scale_drift.txt'))
      scale_drift[:,0] = scale_drift[:,0]-scale_drift[0,0]
      ax_scale.plot(scale_drift[:,0], scale_drift[:,1], label=params['experiment_label'])
  
    ax_scale.set_xlim([0, rot_error[-1,0]+4])
    ax_scale.legend(bbox_to_anchor=[0, 1], loc='upper left', ncol=3)
    ax_scale.grid()
    fig_scale.tight_layout()
    fig_scale.savefig(os.path.join(comparison_dir, 'scale_drift.pdf'))
  
  # ------------------------------------------------------------------------------
  # trajectory
  
#   fig_traj = plt.figure(figsize=(8,4.8))
#   ax_traj = fig_traj.add_subplot(111, xlabel='x [m]', ylabel='y [m]', aspect='equal', xlim=[-3.1, 4], ylim=[-1.5, 2.6])
# 
#   plotTrajectory(ax_traj, '/home/cforster/Datasets/asl_vicon_d2/groundtruth_filtered.txt', 'Groundtruth', 'k', 1.5)
#   plotTrajectory(ax_traj, results_dir+'/20130911_2229_nslam_i7_asl2_fast/traj_estimate_rotated.txt', 'Fast', 'g', 1)
#   plotTrajectory(ax_traj, results_dir+'/20130906_2149_ptam_i7_asl2/traj_estimate_rotated.txt', 'PTAM', 'r', 1)
# 
#   mark_inset(ax_traj, axins, loc1=2, loc2=4, fc="none", ec='b')
#   plt.draw()
#   plt.show() 
#   ax_traj.legend(bbox_to_anchor=[1, 0], loc='lower right', ncol=3)
#   ax_traj.grid()
#   fig_traj.tight_layout()
#   fig_traj.savefig('../results/trajectory_asl.pdf')
  
  
if __name__ == '__main__':
  
  default_name = time.strftime("%Y%m%d_%H%M", time.localtime())+'_comparison'
  parser = argparse.ArgumentParser(description='Compare results.')
  parser.add_argument('result_directories', nargs='+', help='list of result directories to compare')
  parser.add_argument('--name', help='name of the comparison', default=default_name)
  args = parser.parse_args()
 
  # create folder for comparison results
  results_dir = os.path.join(rospkg.RosPack().get_path('svo_analysis'), 'results')
  comparison_dir = os.path.join(results_dir, args.name)
  if not os.path.exists(comparison_dir):
    os.makedirs(comparison_dir)
    
  # run comparison
  compare_results(args.result_directories, results_dir, comparison_dir) 
  
 
                 
           
