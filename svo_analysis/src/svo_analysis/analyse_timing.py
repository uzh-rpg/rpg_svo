#!/usr/bin/python

import os
import numpy as np
import matplotlib.pyplot as plt

def analyse_timing(D, trace_dir):

  # identify measurements which result from normal frames and which from keyframes
  is_frame = np.argwhere(D['repr_n_mps'] >= 0)
  n_frames = len(is_frame)
  
  # set initial time to zero
  D['timestamp'] = D['timestamp'] - D['timestamp'][0]

  # ----------------------------------------------------------------------------
  # plot total time for frame processing
  avg_time = np.mean(D['tot_time'][is_frame])*1000;
  fig = plt.figure(figsize=(8, 3))
  ax = fig.add_subplot(111, ylabel='processing time [ms]', xlabel='time [s]')
  ax.plot(D['timestamp'][is_frame], D['tot_time'][is_frame]*1000, 'g-', label='total time [ms]')
  ax.plot(D['timestamp'][is_frame], np.ones(n_frames)*avg_time, 'b--', label=str('%(time).1fms mean time' % {'time': avg_time}))
  ax.legend()
  fig.tight_layout()
  fig.savefig(os.path.join(trace_dir,'timing.pdf'), bbox_inches="tight")
  
  # ----------------------------------------------------------------------------
  # plot boxplot
  fig = plt.figure(figsize=(6,2))
  ax = fig.add_subplot(111, xlabel='Processing time [ms]')
  ax.boxplot([ 
                D['tot_time'][is_frame]*1000,
#                D['t_local_ba'][is_kf]*1000,
                D['pose_optimizer'][is_frame]*1000 + D['point_optimizer'][is_frame]*1000,
                D['reproject'][is_frame]*1000,
                D['sparse_img_align'][is_frame]*1000,
                D['pyramid_creation'][is_frame]*1000
              ], 0,'', vert=0) 
               
  boxplot_labels = [
                    r'\textbf{Total Motion Estimation: %2.2fms}' % np.median(D['tot_time'][is_frame]*1000),
#                    'Local BA (KF only): %.2fms ' % np.median(D['local_ba'][is_kf]*1000),
                    'Refinement: %2.2fms' % np.median(D['pose_optimizer'][is_frame]*1000 + D['point_optimizer'][is_frame]*1000),
                    'Feature Alignment: %2.2fms' % np.median(D['reproject'][is_frame]*1000),
                    'Sparse Image Alignment: %2.2fms' % np.median(D['sparse_img_align'][is_frame]*1000),
                    'Pyramid Creation: %2.2fms' % np.median(D['pyramid_creation'][is_frame]*1000) ]
                   
  ax.set_yticks(np.arange(len(boxplot_labels))+1)
  ax.set_yticklabels(boxplot_labels)
  fig.tight_layout()
  fig.savefig(os.path.join(trace_dir,'timing_boxplot.pdf'), bbox_inches="tight")
  
  # ----------------------------------------------------------------------------
  # plot boxplot reprojection
  fig = plt.figure(figsize=(6,2))
  ax = fig.add_subplot(111, xlabel='Processing time [ms]')
  ax.boxplot([ D['reproject'][is_frame]*1000,
               D['feature_align'][is_frame]*1000,
               D['reproject_candidates'][is_frame]*1000,
               D['reproject_kfs'][is_frame]*1000 ], 0, '', vert=0) 
  boxplot_labels = [r'\textbf{Total Reprojection: %2.2fms}' % np.median(D['reproject'][is_frame]*1000),
                     'Feature Alignment: %2.2fms' % np.median(D['feature_align'][is_frame]*1000),
                     'Reproject Candidates: %2.2fms' % np.median(D['reproject_candidates'][is_frame]*1000),
                     'Reproject Keyframes: %2.2fms' % np.median(D['reproject_kfs'][is_frame]*1000) ]             
  ax.set_yticks(np.arange(len(boxplot_labels))+1)
  ax.set_yticklabels(boxplot_labels)
  fig.tight_layout()
  fig.savefig(os.path.join(trace_dir,'timing_reprojection.pdf'), bbox_inches="tight")