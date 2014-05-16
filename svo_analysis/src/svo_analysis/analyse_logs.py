#!/usr/bin/python

import os
import yaml
import numpy as np
import matplotlib.pyplot as plt
  
def analyse_logs(D, trace_dir):

  # identify measurements which result from normal frames and which from keyframes
  is_kf = np.argwhere( (D['dropout'] == 1) & (D['repr_n_mps'] >= 0))
  is_frame = np.argwhere(D['repr_n_mps'] >= 0)
  is_nokf = np.argwhere( (D['dropout'] == 0) & (D['repr_n_mps'] >= 0))
   
  # set initial time to zero
  D['timestamp'] = D['timestamp'] - D['timestamp'][0]
  
  # ----------------------------------------------------------------------------
  # plot number of reprojected points
  mean_n_reproj_points = np.mean(D['repr_n_mps'][is_frame]);
  mean_n_reproj_matches = np.mean(D['repr_n_new_references'][is_frame]);
  mean_n_edges_final = np.mean(D['sfba_n_edges_final'][is_frame]);

  fig = plt.figure(figsize=(8,3))
  ax = fig.add_subplot(111, xlabel='time [s]')
  ax.plot(D['timestamp'][is_frame], D['repr_n_mps'][is_frame], 'r-',
          label='Reprojected Points, avg = %.2f'%mean_n_reproj_points)
  ax.plot(D['timestamp'][is_frame], D['repr_n_new_references'][is_frame], 'b-', 
          label='Feature Matches, avg = %.2f'%mean_n_reproj_matches)
  ax.plot(D['timestamp'][is_frame], D['sfba_n_edges_final'][is_frame], 'g-', 
          label='Points after Optimization, avg = %.2f'%mean_n_edges_final)
  ax.set_ylim(bottom=0)
  ax.legend(loc='lower right')
  fig.tight_layout()
  fig.savefig(os.path.join(trace_dir,'num_reprojected.pdf'), bbox_inches="tight")

  # ----------------------------------------------------------------------------
  # plot median error before and after pose-optimzation and bundle adjustment
  init_error_avg = np.mean(D['sfba_error_init'][is_frame])
  opt1_avg = np.mean(D['sfba_error_final'][is_frame])

  fig = plt.figure(figsize=(8,2))
  ax = fig.add_subplot(111, xlabel='time [s]', ylabel='error [px]')
  ax.plot(D['timestamp'][is_frame], D['sfba_error_init'][is_frame], 'r-', label='Initial error')
  ax.plot(D['timestamp'][is_frame], D['sfba_error_final'][is_frame], 'b-', label='Final error')
  ax.legend(ncol=2)  
  fig.tight_layout()
  fig.savefig(os.path.join(trace_dir,'reprojection_error.pdf'), bbox_inches="tight")
  print 'average reprojection error improvement: ' + str(init_error_avg - opt1_avg)

  # ----------------------------------------------------------------------------
  # plot number of candidate points
  fig = plt.figure(figsize=(8,3))
  ax = fig.add_subplot(111, xlabel='time [s]')
  ax.plot(D['timestamp'][is_frame], D['n_candidates'][is_frame], 'r-', label='Candidate Points')
  fig.tight_layout()
  fig.savefig(os.path.join(trace_dir,'candidate_points.pdf'), bbox_inches="tight")
  
  # ----------------------------------------------------------------------------
  # plot number of candidate points
  fig = plt.figure(figsize=(8,2))
  ax = fig.add_subplot(111, xlabel='time [s]', ylabel='px')
  ax.plot(D['timestamp'][is_frame], D['sfba_thresh'][is_frame], 'r-', label='Threshold')
  fig.tight_layout()
  fig.savefig(os.path.join(trace_dir,'optimization_thresh.pdf'), bbox_inches="tight")
  
  # ----------------------------------------------------------------------------
  # write other statistics to file
  stat = {'num_frames': len(is_frame), 
          'num_kfs': len(is_kf),
          'reproj_error_avg_improvement': float(init_error_avg - opt1_avg)}
  with open(os.path.join(trace_dir,'dataset_stats.yaml'),'w') as outfile:          
    outfile.write(yaml.dump(stat, default_flow_style=False))

