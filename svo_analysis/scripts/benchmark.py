#!/usr/bin/python
"""
Created on Sat Aug 10 18:21:47 2013

@author: Christian Forster
"""

import os
import yaml
import rospkg
import argparse
import time
import vikit_py.cpu_info as cpu_info
import vikit_py.ros_node as ros_node
import evaluate

def run_experiment(dataset, params):
    # load dataset parameters
    params['dataset_directory'] = os.path.join(os.environ['SVO_DATASET_DIR'], dataset)
    if not os.path.exists(params['dataset_directory']):
        raise Exception("Provided dataset folder does not exist.")
    dataset_params_file = os.path.join(params['dataset_directory'], 'dataset_params.yaml')
    dataset_params = yaml.load(open(dataset_params_file,'r'))
      
    # load algorithm parameters
    algo_params_file = os.path.join(rospkg.RosPack().get_path('svo_ros'), 
                                   'param', params['param_settings']+'.yaml')
    algo_params = yaml.load(open(algo_params_file,'r'))
    
    # combine all parameters
    params = dict(params.items() + algo_params.items() + dataset_params['rig']['cam0'].items())
      
    # dump experiment params to file:
    params_dump_file = params['trace_dir']+'/params.yaml'
    with open(params_dump_file,'w') as outfile:
        outfile.write(yaml.dump(params, default_flow_style=False))
    
    # execute ros node
    node = ros_node.RosNode(args.version, args.executable)
    node.run(params)
    
if __name__=="__main__":
  
    # parse command line
    parser = argparse.ArgumentParser(description='''
    Runs SVO with the dataset and parameters specified in the provided experiment file.
    ''')
    parser.add_argument('experiment_file', help='experiment file in svo_analysis/experiments folder')
    parser.add_argument('--evaluate', help='evaluate tracefile after running SVO', action='store_true')
    parser.add_argument('--version', help='version of svo to evaluate', default='svo_ros')
    parser.add_argument('--executable', help='the executable to be called', default='benchmark')
    args = parser.parse_args()
    
    # load experiment parameters
    experiment_params_file = os.path.join(rospkg.RosPack().get_path('svo_analysis'), 
                               'experiments', args.experiment_file+'.yaml')
    experiment_params = yaml.load(open(experiment_params_file, 'r'))
    params = dict()
    params['experiment_label'] = experiment_params['experiment_label']
    params['param_settings'] = experiment_params['param_settings']
    params['time'] = time.strftime("%Y%m%d_%H%M", time.localtime())
    params['platform'] = cpu_info.get_cpu_info()
    params['trace_name'] = 'trace'
    
    trace_dirs = list()
    for dataset in experiment_params['datasets']:
        params['experiment_name'] = params['time']+'_'+args.version+'_'+args.experiment_file+'_'+dataset
        params['trace_dir'] = os.path.join(rospkg.RosPack().get_path('svo_analysis'),
                                           'results', params['experiment_name'])
        if not os.path.exists(params['trace_dir']):
            os.makedirs(params['trace_dir'])
        trace_dirs.append(params['trace_dir'])
        run_experiment(dataset, params)
        
    # TODO: check if it is a synthetic dataset 
    for trace in trace_dirs:
        evaluate.evaluate_dataset(trace)
      
      
  
  
  
  
