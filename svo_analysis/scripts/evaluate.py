#!/usr/bin/python
"""
Created on Sat Aug 10 18:21:47 2013

@author: Christian Forster
"""

import os
import csv
import numpy as np
import rospkg
import argparse
import yaml
import svo_analysis.analyse_logs as analyse_logs
import svo_analysis.analyse_timing as analyse_timing
import svo_analysis.analyse_trajectory as analyse_trajectory
import svo_analysis.analyse_depth as analyse_depth

def evaluate_dataset(trace_dir):
  
    # read tracefile
    data = csv.reader(open(os.path.join(trace_dir, 'trace.csv')))
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

    # generate plots
    analyse_logs.analyse_logs(D, trace_dir)
    analyse_timing.analyse_timing(D, trace_dir)
    analyse_trajectory.analyse_trajectory(trace_dir)

    param = yaml.load(open(os.path.join(trace_dir, 'dataset_params.yaml'), 'r'))
    if param['dataset_is_blender']:
        analyse_depth.analyse_depth(trace_dir)
        analyse_depth.analyse_depth_over_time(trace_dir)


if __name__=="__main__":
    parser = argparse.ArgumentParser(description='''
    Evaluates tracefiles of SVO and generates the plots.
    ''')
    parser.add_argument('experiment_name', help='directory name of the tracefiles')
    args = parser.parse_args()

    trace_dir = os.path.join(rospkg.RosPack().get_path('svo_analysis'), 
                             'results',
                             args.experiment_name)
    evaluate_dataset(trace_dir)