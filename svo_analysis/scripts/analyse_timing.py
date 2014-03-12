#!/usr/bin/python

import argparse
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc

def analyse_timing(D, trace_name):

  # identify measurements which result from normal frames and which from keyframes
  is_frame = np.argwhere(D['repr_n_mps'] >= 0)
  n_frames = len(is_frame)

  # plot total time for frame processing
  avg_time = np.mean(D['t_tot_time'][is_frame])*1000;

  fig = plt.figure(figsize=(8, 3))
  ax = fig.add_subplot(111, ylabel='processing time [ms]', xlabel='time [s]')
  ax.plot(np.arange(n_frames), D['t_tot_time'][is_frame]*1000, 'g-', label='total time [ms]')
  ax.plot(np.arange(n_frames), np.ones(n_frames)*avg_time, 'b--', label=str('%(time).1fms mean time' % {'time': avg_time}))
  ax.legend()
  fig.tight_layout()
  fig.savefig(trace_name+'_img_tot_processing_time.pdf', bbox_inches="tight")
  
  fig = plt.figure(figsize=(6,2))
  ax = fig.add_subplot(111, xlabel='Processing time [ms]')
  ax.boxplot([ 
                D['t_tot_time'][is_frame]*1000,
#                D['t_local_ba'][is_kf]*1000,
                D['t_pose_optimizer'][is_frame]*1000 + D['t_point_optimizer'][is_frame]*1000,
                D['t_reproject'][is_frame]*1000,
                D['t_sparse_img_align'][is_frame]*1000,
                D['t_pyramid_creation'][is_frame]*1000
              ], 0,'', vert=0) # 
               
  boxplot_labels = [
                    r'\textbf{Total Motion Estimation: %2.2fms}' % np.median(D['t_tot_time'][is_frame]*1000),
#                    'Local BA (KF only): %.2fms ' % np.median(D['t_local_ba'][is_kf]*1000),
                    'Refinement: %2.2fms' % np.median(D['t_pose_optimizer'][is_frame]*1000 + D['t_point_optimizer'][is_frame]*1000),
                    'Feature Alignment: %2.2fms' % np.median(D['t_reproject'][is_frame]*1000),
                    'Sparse Image Alignment: %2.2fms' % np.median(D['t_sparse_img_align'][is_frame]*1000),
                    'Pyramid Creation: %2.2fms' % np.median(D['t_pyramid_creation'][is_frame]*1000) ]
                   
  ax.set_yticks(np.arange(len(boxplot_labels))+1)
  ax.set_yticklabels(boxplot_labels)
  fig.tight_layout()
  fig.savefig(trace_name+'_timing_boxplot.pdf', bbox_inches="tight")
  
  fig = plt.figure(figsize=(8, 5))
  ax = fig.add_subplot(111, ylabel='time [ms]')
  ax.plot(D['t_reproject']*1000, 'r', label='reproject')
  ax.plot(D['t_reproject_find_pts']*1000, 'g', label='reproject find pts') # ok!
  ax.plot(D['t_reproject_best']*1000, 'm', label='reproject best')
  ax.plot(D['t_reproject_candidates']*1000, 'b', label='reproject candidates')
  ax.legend()
  fig.tight_layout()
  fig.savefig(trace_name+'_reprojection.pdf', bbox_inches="tight")


if __name__=="__main__":

  # config
  dataset='20131003_1937_nslam_i7_asl2_fast'
  results_dir = '../results/'+dataset
  tracefile = results_dir + '/'+dataset+'_0.csv'
  
  # parse command line
  parser = argparse.ArgumentParser(description='''
  This script computes the absolute trajectory error from the ground truth trajectory and the estimated trajectory.
  ''')
  parser.add_argument('--trace_file', help='nanoslam tracefile with timings and logs', default=tracefile)
  parser.add_argument('--trace_name', help='name of the tracefile, used for saving figures etc.', default=results_dir)
  args = parser.parse_args()
  results_dir = args.trace_name
  
  # tracefile
  data = csv.reader(open(args.trace_file))
  fields = data.next()
  D = dict()
  for field in fields:
    D[field] = list()

  # fill dictionary with column values
  for row in data:
    for (field, value) in zip(fields, row):
      D[field].append(float(value))

  # change dictionary values from list to numpy array for easier manipulation
  for field, value in D.items():
    D[field] = np.array(D[field])

  # tell matplotlib to use latex font
  rc('font',**{'family':'serif','serif':['Cardo']})
  rc('text', usetex=True)

  