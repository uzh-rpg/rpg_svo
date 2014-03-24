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
import svo_analysis.cpu_info as cpu_info

class RosNode:
  def __init__(self, package, executable):
    self.package = package
    self.executable = executable
    
  def run(self, parameter_dictionary):
    parameter_string = ''
    for key in parameter_dictionary.keys():
      parameter_string = parameter_string + ' _' + key + ':=' + str(parameter_dictionary[key])
    print('starting ros node')
    os.system('rosrun ' + self.package + ' ' + self.executable + ' ' + parameter_string)
    print('ros node finished processing')

if __name__=="__main__":
  
  # parse command line
  parser = argparse.ArgumentParser(description='''
  Runs SVO with the dataset and parameters specified in the provided experiment file.
  ''')
  parser.add_argument('experiment_file', help='experiment file in svo_analysis/experiments folder')
  parser.add_argument('--name', help='experiment name')
  parser.add_argument('--evaluate', help='evaluate tracefile after running SVO', action='store_true')
  args = parser.parse_args()

  # load experiment parameters
  expParamsFile = os.path.join(rospkg.RosPack().get_path('svo_analysis'), 
                               'experiments', args.experiment_file+'.yaml')
  expParams = yaml.load(open(expParamsFile, 'r'))
  expParams['time'] = time.strftime("%Y%m%d_%H%M", time.localtime())
  #expParams['platform'] = cpu_info.getCpuInfo()
  expParams['experiment_name'] = args.name
  if(expParams['experiment_name'] == None):
    expParams['experiment_name'] = expParams['time']+'_svo_' + args.experiment_file

  # load dataset parameters
  datasetParamsFile = os.path.join(rospkg.RosPack().get_path('Datasets'), 
                                   expParams['dataset'], 'dataset_params.yaml')
  datasetParams = yaml.load(open(datasetParamsFile,'r'))
  
  # load algorithm parameters
  algoParamsFile = os.path.join(rospkg.RosPack().get_path('svo_ros'), 
                                   'param', expParams['param_settings']+'.yaml')
  algoParams = yaml.load(open(algoParamsFile,'r'))

  # combine all parameters
  params = dict(expParams.items() + datasetParams.items() + algoParams.items())
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
    
  # start ros node
  node = RosNode('svo_ros','benchmark')
  node.run(params)
  
  
  
