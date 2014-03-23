#!/usr/bin/python

import argparse
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc

def analyse_logs(D, trace_name):

  # identify measurements which result from normal frames and which from keyframes
  is_kf = np.argwhere( (D['dropout'] == 1) & (D['repr_n_mps'] >= 0))
  is_frame = np.argwhere(D['repr_n_mps'] >= 0)
  is_nokf = np.argwhere( (D['dropout'] == 0) & (D['repr_n_mps'] >= 0))
  n_frames = len(is_frame)
  n_kfs = len(is_kf)
  n_nokf = len(is_nokf)
  fps = 25.0
  
  # plot number of reprojected points
  mean_n_reproj_points = np.mean(D['repr_n_mps'][is_frame]);
  mean_n_reproj_matches = np.mean(D['repr_n_new_references'][is_frame]);

  plt.figure()
  plt.plot(np.arange(n_frames), D['repr_n_mps'][is_frame], 'r-', label='reprojected points, avg = %(i).2f'%{'i':mean_n_reproj_points})
  plt.plot(np.arange(n_frames), D['repr_n_new_references'][is_frame], 'b-', label='reprojected matches, avg = %(i).2f'%{'i':mean_n_reproj_matches})
  plt.legend()
  plt.tight_layout()
  plt.title('Reprojected points asdf')
  plt.savefig(trace_name+'_reprojection.pdf', bbox_inches="tight")

  # plot number of triangulations
#  plt.figure(3)
#  plt.plot(np.arange(n_kfs), D['triang_n_points'][is_kf], 'g-', label='triangulated points, avg %0.2f'%np.mean(D['triang_n_points'][is_kf]))
#  plt.plot(np.arange(n_kfs), D['triang_n_refs'][is_kf], 'b-', label='new references, avg %0.2f'%np.mean(D['triang_n_refs'][is_kf]))
#  plt.plot(np.arange(n_kfs), D['triang_n_fails'][is_kf], 'r-', label='failures, avg %0.2f'%np.mean(D['triang_n_fails'][is_kf]))
#  plt.plot(np.arange(n_kfs), D['triang_n_add_refs'][is_kf], 'm-', label='additional refs, avg %0.2f'%np.mean(D['triang_n_add_refs'][is_kf]))
#  plt.legend()
#  plt.title('Triangulation')
#  plt.savefig(trace_name+'_img_triangulation.png', bbox_inches="tight")

  # plot number of klt tracked and matched points
  fig = plt.figure(figsize=(8,2))
  ax = fig.add_subplot(111, xlabel='time [s]', ylabel='no. features')
  ax.plot(np.arange(n_frames)/fps, D['repr_n_new_references'][is_frame], 'b-', label='number of successful matches')
  fig.tight_layout()
  fig.savefig(trace_name+'number_of_features.pdf', bbox_inches="tight")

  # plot median error before and after pose-optimzation and bundle adjustment
  init_error_avg = np.mean(D['sfba_error_init'][is_frame])
  opt1_avg = np.mean(D['sfba_error_final'][is_frame])

  fig = plt.figure(figsize=(8,2))
  ax = fig.add_subplot(111, xlabel='time [s]', ylabel='error [px]')
  t_max = n_frames
  ax.plot(np.arange(n_frames)[0:t_max]/fps, D['sfba_error_init'][is_frame][0:t_max], 'r-', label='Initial error')
  ax.plot(np.arange(n_frames)[0:t_max]/fps, D['sfba_error_final'][is_frame][0:t_max], 'b-', label='Final error')
  ax.legend(ncol=2)  
  ax.set_xlim([0, t_max/fps+4])
  #ax.set_ylim([0, 0.8])
  fig.tight_layout()
  fig.savefig(trace_name+'reprojection_error.pdf', bbox_inches="tight")
  print 'average reprojection error improvement: ' + str(init_error_avg - opt1_avg)

  # plot number of references
  plt.figure()
  plt.fill(np.arange(n_frames+1), np.append(D['repr_n_new_references'][is_frame]+D['klt_n_matches'][is_frame],0), 'g-', label='Reprojection')
  plt.fill(np.arange(n_frames+1), np.append(D['klt_n_tracked'][is_frame],0), 'b-', label='KLT Matches')
  plt.plot(np.arange(n_frames), D['sfba_n_edges_final'][is_frame], 'r-', label='References after Optimization', linewidth=2.0)
  plt.legend()
  plt.tight_layout()
  plt.title('Pose Optimization')
  plt.savefig(trace_name+'_img_n_references.pdf', bbox_inches="tight")
  
  # plot number of seeds
  plt.figure()
  has_seed = np.argwhere(D['seeds_n_tot'] >= 0)
  plt.fill(np.arange(len(has_seed)+1), np.append(D['seeds_n_tot'][has_seed],0), label='nSeeds', edgecolor='none') 
  plt.fill(np.arange(len(has_seed)+1), np.append(D['seeds_n_updates'][has_seed]+D['seeds_n_failed_matches'][has_seed],0), label='nFailedMatches', edgecolor='none')
  plt.fill(np.arange(len(has_seed)+1), np.append(D['seeds_n_updates'][has_seed],0), label='nUpdates', edgecolor='none') 
  plt.legend()
  plt.tight_layout()
  plt.title('Seeds')
  plt.savefig(trace_name+'_seeds.pdf', bbox_inches="tight")


  plt.figure()
  plt.plot(D['n_candidates'])
  plt.title('Candidates')
  plt.show()
  
#  plt.figure()
#  plt.plot())

if __name__=="__main__":

  dataset='20131003_1937_nslam_i7_asl2_fast'
  n_align_frames = 10
  save = True
  results_dir = '../results/'+dataset
  logfile = results_dir+'/'+dataset+'_0.csv'
  
  # parse command line 
  parser = argparse.ArgumentParser(description='''
  This script computes the absolute trajectory error from the ground truth trajectory and the estimated trajectory.
  ''')
  parser.add_argument('--trace_file', help='nanoslam tracefile with timings and logs', default=logfile)
  parser.add_argument('--trace_name', help='name of the tracefile, used for saving figures etc.', default=results_dir+'/')
  args = parser.parse_args()

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

  analyse_logs(D, args.trace_name)

