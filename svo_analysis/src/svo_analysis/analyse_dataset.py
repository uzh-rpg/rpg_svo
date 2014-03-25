# -*- coding: utf-8 -*-

import associate
import numpy as np
import matplotlib.pyplot as plt
import yaml

def loadDataset(filename):
  file = open(filename)
  data = file.read()
  lines = data.replace(","," ").replace("\t"," ").split("\n")
  D = np.array([[v.strip() for v in line.split(" ") if v.strip()!=""] for line in lines if len(line)>0 and line[0]!="#"], dtype=np.float64)
  return D
  
dataset_dir = '/home/cforster/Datasets/SlamBenchmark/px4_r2'
trajectory_data = dataset_dir+'/groundtruth.txt'
stepsize = 10

# load dataset
data = loadDataset(trajectory_data)
n = data.shape[0]
steps = np.arange(0,n,stepsize)

# compute trajectory length
last_pos = data[0,1:4]
trajectory_length = 0

for i in steps[1:]:
  new_pos = data[i,1:4]
  trajectory_length += np.linalg.norm(new_pos-last_pos)
  last_pos = new_pos
  

print 'trajectory lenght = ' + str(trajectory_length) + 'm'
print 'height mean = ' + str(np.mean(data[:,3])) + 'm'
print 'height median = ' + str(np.median(data[:,3])) + 'm'
print 'height std = ' + str(np.std(data[:,3])) + 'm'
print 'duration = ' + str(data[-1,0]-data[0,0]) + 's'
print 'speed = ' + str(trajectory_length/(data[-1,0]-data[0,0])) + 'm/s'

  
