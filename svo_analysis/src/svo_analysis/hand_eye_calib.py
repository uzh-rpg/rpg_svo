# -*- coding: utf-8 -*-
"""
Created on Sun Aug  4 15:47:55 2013

@author: cforster
"""

import yaml
import numpy as np
import associate
import rotation_utils as ru
import matplotlib.pyplot as plt

# user config
display = True
dataset = '20130906_2149_ptam_i7_asl2'
n_measurements = 1000
delta = 20

# load dataset parameters
dataset_dir = '../results/'+dataset
params_stream = open(dataset_dir+'/params.yaml')
params = yaml.load(params_stream)

# set trajectory groundtruth and estimate file
traj_groundtruth = params['dataset_directory']+'/groundtruth_filtered.txt'
traj_estimate = dataset_dir+'/traj_estimate.txt'

# load dataset params
dataset_param_file_stream = open(params['dataset_directory']+'/dataset_params.yaml','r')
dataset_param = yaml.load(dataset_param_file_stream)

# load data
data_gt = associate.read_file_list(traj_groundtruth)
data_es = associate.read_file_list(traj_estimate)

# select matches
#offset = -dataset_param['cam_delay']
offset = -0.03
matches = associate.associate(data_gt, data_es, offset, 0.02)
#matches = matches[0:470]
p_gt = np.array([[float(value) for value in data_gt[a][0:3]] for a,b in matches])
q_gt = np.array([[float(value) for value in data_gt[a][3:7]] for a,b in matches])
p_es = np.array([[float(value) for value in data_es[b][0:3]] for a,b in matches])
q_es = np.array([[float(value) for value in data_es[b][3:7]] for a,b in matches])

# --------------------------------------------------------------------------------
# align Sim3 to get scale
scale,rot,trans = align_sim3(p_gt, p_es, np.shape(matches)[0])
p_es = scale*p_es
#p_es = np.transpose(scale*np.dot(rot,np.transpose(p_es)))+trans

print 's='+str(scale)
#print 't='+str(trans)
#print 'R='+str(rot)

# plot sim3 aligned trajectory
if display:
  fig = plt.figure()
  ax = fig.add_subplot(111, aspect='equal')
  ax.plot(p_es[:,0], p_es[:,1], 'r-', label='estimate')
  ax.plot(p_gt[:,0], p_gt[:,1], 'b-', label='groundtruth')
  ax.legend()

# --------------------------------------------------------------------------------
# hand-eye-calib

# select random measurements
I = np.array(rand(n_measurements,1)*(np.shape(matches)[0]-delta), dtype=int)[:,0]
R,b = hand_eye_calib(q_gt, q_es, p_gt, p_es, I, delta, True)

print 'quat = ' + str(ru.dcm2quat(R))
print 'b = ' + str(b)

rpy_es = np.zeros([q_es.shape[0]-1, 3])
rpy_gt = np.zeros([q_gt.shape[0]-1, 3])
t_gt = np.zeros([q_es.shape[0]-1,3])
t_es = np.zeros([q_es.shape[0]-1,3])

for i in range(delta,np.shape(q_es)[0]):
  A1 = ru.quat2dcm(q_es[i-delta,:])
  A2 = ru.quat2dcm(q_es[i,:])
  A  = np.dot(A1.transpose(), A2)
  B1 = ru.quat2dcm(q_gt[i-delta,:])
  B2 = ru.quat2dcm(q_gt[i,:])
  B  = np.dot(B1.transpose(), B2)
  B_es  = np.dot(np.transpose(R), np.dot(A, R))
  rpy_gt[i-delta,:] = ru.dcm2rpy(B)
  rpy_es[i-delta,:] = ru.dcm2rpy(B_es)
  t_B = np.dot(np.transpose(B1),(p_gt[i,:]-p_gt[i-delta,:]))
  t_A = np.dot(np.transpose(A1),(p_es[i,:]-p_es[i-delta,:]))
  t_gt[i-delta,:] = t_B
  t_es[i-delta,:] = np.dot(np.transpose(R), np.dot(A,b[:,0]) + t_A - b[:,0])

alignment_error = (t_gt-t_es)
error = np.sqrt(np.sum(np.multiply(alignment_error,alignment_error),1))
I_accurate = np.argwhere(error < np.percentile(error, 90))[:,0]

if display:
  plt.figure()
  plt.plot(rpy_es[:,0], 'r-', label='es roll')
  plt.plot(rpy_es[:,1], 'g-', label='es pitch')
  plt.plot(rpy_es[:,2], 'b-', label='es yaw')
  plt.plot(rpy_gt[:,0], 'r--', label='gt roll')
  plt.plot(rpy_gt[:,1], 'g--', label='gt pitch')
  plt.plot(rpy_gt[:,2], 'b--', label='gt yaw')
  plt.legend()
  plt.figure()
  plt.plot(t_gt[:,0], 'r-', label='gt x')
  plt.plot(t_gt[:,1], 'g-', label='gt y')
  plt.plot(t_gt[:,2], 'b-', label='gt z')
  plt.plot(t_es[:,0], 'r--', label='es x')
  plt.plot(t_es[:,1], 'g--', label='es y')
  plt.plot(t_es[:,2], 'b--', label='es z')
  plt.legend()
  plt.figure()
  plt.plot(error,'-g')


print 'e_rms = ' + str(np.sqrt(np.dot(error,error) / len(error)))
print 'e_mean = ' + str(np.mean(error))
print 'e_median = ' + str(np.median(error))
print 'e_std = ' + str(np.std(error))


# now sample again from the filtered list:
N = 10
display = False
n_acc_meas = I_accurate.size
n_measurements = 500

#for i in range(5):
#  print '-------------------------------------'
#  i0 = np.array(rand(n_measurements,1)*np.shape(I_accurate)[0], dtype=int)[:,0]
#  i1 = np.minimum(i0+N, n_acc_meas-1)
#  I = np.empty(i0.size*2, dtype=int)
#  I[0::2] = I_accurate[i0]
#  I[1::2] = I_accurate[i1]
#  R,b = handEyeCalib(q_gt[I,:], q_es[I,:], p_gt[I,:], p_es[I,:], True)
#  print 'quat = ' + str(ru.dcm2quat(R))
#  print 'b = ' + str(b)
#  rpy_es = np.zeros([q_es.shape[0]-1, 3])
#  rpy_gt = np.zeros([q_gt.shape[0]-1, 3])
#  t_gt = np.zeros([q_es.shape[0]-1,3])
#  t_es = np.zeros([q_es.shape[0]-1,3])
#
#  delta = 10
#  for i in range(delta,np.shape(q_es)[0]):
#    A1 = ru.quat2dcm(q_es[i-delta,:])
#    A2 = ru.quat2dcm(q_es[i,:])
#    A  = np.dot(A1.transpose(), A2)
#    B1 = ru.quat2dcm(q_gt[i-delta,:])
#    B2 = ru.quat2dcm(q_gt[i,:])
#    B  = np.dot(B1.transpose(), B2)
#    B_es  = np.dot(np.transpose(R), np.dot(A, R))
#    rpy_gt[i-delta,:] = ru.dcm2rpy(B)
#    rpy_es[i-delta,:] = ru.dcm2rpy(B_es)
#    t_B = np.dot(np.transpose(B1),(p_gt[i,:]-p_gt[i-delta,:]))
#    t_A = np.dot(np.transpose(A1),(p_es[i,:]-p_es[i-delta,:]))
#    t_gt[i-delta,:] = t_B
#    t_es[i-delta,:] = np.dot(np.transpose(R), np.dot(A,b[:,0]) + t_A - b[:,0])
#  alignment_error = (t_gt-t_es)
#  error = np.sqrt(np.sum(np.multiply(alignment_error,alignment_error),1))
#
#  if display:
#    plt.figure()
#    plt.plot(rpy_es[:,0], 'r-', label='es roll')
#    plt.plot(rpy_es[:,1], 'g-', label='es pitch')
#    plt.plot(rpy_es[:,2], 'b-', label='es yaw')
#    plt.plot(rpy_gt[:,0], 'r--', label='gt roll')
#    plt.plot(rpy_gt[:,1], 'g--', label='gt pitch')
#    plt.plot(rpy_gt[:,2], 'b--', label='gt yaw')
#    plt.legend()
#    plt.figure()
#    plt.plot(t_gt[:,0], 'r-', label='es x')
#    plt.plot(t_gt[:,1], 'g-', label='es y')
#    plt.plot(t_gt[:,2], 'b-', label='es z')
#    plt.plot(t_es[:,0], 'r--', label='gt x')
#    plt.plot(t_es[:,1], 'g--', label='gt y')
#    plt.plot(t_es[:,2], 'b--', label='gt z')
#    plt.legend()
#    plt.figure()
#    plt.plot(error,'-g')
#
#  print 'e_rms = ' + str(np.sqrt(np.dot(error,error) / len(error)))
#  print 'e_mean = ' + str(np.mean(error))
#  print 'e_median = ' + str(np.median(error))
#  print 'e_std = ' + str(np.std(error))
























