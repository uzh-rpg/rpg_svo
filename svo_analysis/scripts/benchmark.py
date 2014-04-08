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

if __name__=="__main__":
  
  # parse command line
  parser = argparse.ArgumentParser(description='''
  Runs SVO with the dataset and parameters specified in the provided experiment file.
  ''')
  parser.add_argument('experiment_file', help='experiment file in svo_analysis/experiments folder')
  parser.add_argument('--name', help='experiment name')
  parser.add_argument('--evaluate', help='evaluate tracefile after running SVO', action='store_true')
  parser.add_argument('--version', help='version of svo to evaluate', default='svo_ros')
  parser.add_argument('--executable', help='the executable to be called', default='benchmark')
  args = parser.parse_args()

  # load experiment parameters
  expParamsFile = os.path.join(rospkg.RosPack().get_path('svo_analysis'), 
                               'experiments', args.experiment_file+'.yaml')
  expParams = yaml.load(open(expParamsFile, 'r'))
  expParams['time'] = time.strftime("%Y%m%d_%H%M", time.localtime())
  expParams['platform'] = cpu_info.get_cpu_info()
  expParams['experiment_name'] = args.name
  if(expParams['experiment_name'] == None):
    expParams['experiment_name'] = expParams['time']+'_'+args.version+'_' + args.experiment_file

  # load dataset parameters
  expParams['dataset_directory'] = os.path.join(os.environ['SVO_DATASET_DIR'], expParams['dataset'])
  if not os.path.exists(expParams['dataset_directory']):
    raise Exception("Provided dataset folder does not exist.")
  datasetParamsFile = os.path.join(expParams['dataset_directory'], 'dataset_params.yaml')
  datasetParams = yaml.load(open(datasetParamsFile,'r'))
  
  # load algorithm parameters
  algoParamsFile = os.path.join(rospkg.RosPack().get_path('svo_ros'), 
                                'param', expParams['param_settings']+'.yaml')
  algoParams = yaml.load(open(algoParamsFile,'r'))

  # combine all parameters
  params = dict(expParams.items() + datasetParams.items() + datasetParams['rig']['cam0'].items() + algoParams.items())
  params['trace_name'] = 'trace'
  params['trace_dir'] = os.path.join(rospkg.RosPack().get_path('svo_analysis'),
                                     'results', params['experiment_name'])

      
  
  # create folder for tracing
  if not os.path.exists(params['trace_dir']):
    os.makedirs(params['trace_dir'])
  
  # dump experiment params to file:
  params_dump_file = params['trace_dir']+'/params.yaml'
  with open(params_dump_file,'w') as outfile:
    outfile.write(yaml.dump(params, default_flow_style=False))
    
  # execute ros node
  node = ros_node.RosNode(args.version, args.executable)
  node.run(params)
  
  # TODO: check if it is a synthetic dataset 
  evaluate.evaluate_dataset(params['trace_dir'])
  
  
  
  
  
  
