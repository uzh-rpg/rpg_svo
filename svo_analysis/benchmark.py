#!/usr/bin/python
"""
Created on Sat Aug 10 18:21:47 2013

@author: Christian Forster
"""

import os
import yaml
import time

class RosNode:
  def __init__(self, package, executable):
    self.package = package
    self.executable = executable
    
  def run(self, parameter_dictionary):
    parameter_string = ''
    for key in parameter_dictionary.keys():
      parameter_string = parameter_string + ' _' + key + ':=' + str(parameter_dictionary[key])
    os.system('rosrun ' + self.package + ' ' + self.executable + ' ' + parameter_string)

if __name__=="__main__":
  
  # config:  
  home_dir = os.getenv("HOME")
  dataset_dir = home_dir+'/Datasets/SlamBenchmark/'
  algo_dir = home_dir+'/workspace/rpg_slam/svo_ros'
  trace_dir = home_dir+'/workspace/rpg_slam/svo_analysis/results'
  platform = 'i7'
  
  # load parameters
  experiment_name = 'asl2_fast'
  experiment_file = 'experiments/'+experiment_name+'.yaml'
  experiment_file_stream = open(experiment_file,'r')
  experiment_dict = yaml.load(experiment_file_stream)
  
  dataset_param_file = dataset_dir+experiment_dict['dataset']+'/dataset_params.yaml'
  dataset_param_file_stream = open(dataset_param_file,'r')
  dataset_param_dict = yaml.load(dataset_param_file_stream)
  
  algo_param_file = algo_dir+'/param/'+experiment_dict['param_settings']+'.param'
  algo_param_file_stream = open(algo_param_file,'r')
  algo_param_dict = yaml.load(algo_param_file_stream)
  
  params = dict(dataset_param_dict.items() + algo_param_dict.items() + experiment_dict.items())
  localtime = time.localtime()
  params['trace_name'] = time.strftime("%Y%m%d_%H%M", localtime)+'_nslam_'+platform+'_'+experiment_name
  params['trace_dir'] = trace_dir+'/'+params['trace_name']
  params['run_id'] = 0
  
  # create folder for experiment results
  if not os.path.exists(params['trace_dir']):
    os.makedirs(params['trace_dir'])
    
  # dump experiment params to file:
  params_dump_file = params['trace_dir']+'/params.yaml'
  with open(params_dump_file,'w') as outfile:
    outfile.write( yaml.dump(params, default_flow_style=False) )
    
  # start ros node
  print 'starting ros node'
  node = RosNode('svo_ros','benchmark')
  node.run(params)
  print 'ros node finished processing'
  
  
