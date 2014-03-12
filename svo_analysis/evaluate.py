#!/usr/bin/python
"""
Created on Sat Aug 10 18:21:47 2013

@author: Christian Forster
"""

import argparse
import csv
import numpy as np
from matplotlib import rc
import scripts.analyse_logs as analyse_logs
import scripts.analyse_timing as analyse_timing

if __name__=="__main__":
  
  # parse command line 
  parser = argparse.ArgumentParser()
  parser.add_argument('--trace_name', help='svo tracefile with timings and logs', default='20131121_0930_px7')
  args = parser.parse_args()

  # config
  results_dir = 'results/'+args.trace_name
  trace_file = results_dir+'/'+args.trace_name+'.csv'

  # tracefile
  data = csv.reader(open(trace_file))
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

  analyse_logs.analyse_logs(D, results_dir+'/'+args.trace_name)
  analyse_timing.analyse_timing(D, results_dir+'/'+args.trace_name)