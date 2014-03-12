# -*- coding: utf-8 -*-

import associate
import numpy as np
import matplotlib.pyplot as plt
import yaml
import transformations
from matplotlib import rc

def alignSim3(model, data, n):

  # select the first n datapoints and remove its mean so their center is zero
  M = model[0:n,:]
  D = data[0:n,:]

  # substract mean
  mu_M = M.mean(0)
  mu_D = D.mean(0)
  M_zerocentered = M - mu_M
  D_zerocentered = D - mu_D

  # correlation
  C = 1.0/n*np.dot(M_zerocentered.transpose(), D_zerocentered)
  sigma2 = 1.0/n*np.multiply(D_zerocentered,D_zerocentered).sum()
  U_svd,D_svd,V_svd = np.linalg.linalg.svd(C)
  D_svd = np.diag(D_svd)
  V_svd = np.transpose(V_svd)
  S = np.eye(3)

  if(np.linalg.det(U_svd)*np.linalg.det(V_svd) < 0):
    S[2,2] = -1

  R = np.dot(U_svd, np.dot(S, np.transpose(V_svd)))
  s = 1.0/sigma2*np.trace(np.dot(D_svd, S))
  t = mu_M-s*np.dot(R,mu_D)

  return s, R, t


def getRigidBodyTrafo(quat,trans):
  T = transformations.quaternion_matrix(quat)
  T[0:3,3] = trans
  return T

if __name__ == '__main__':
  
  # config
  dataset='20131003_1937_nslam_i7_asl2_fast'
  n_align_frames = 100
  save = True
  results_dir = '../results/'+dataset
  
  # plot options
  rc('font',**{'family':'serif','serif':['Cardo']})
  rc('text', usetex=True)
  
  # load dataset parameters
  params_stream = open(results_dir+'/params.yaml')
  params = yaml.load(params_stream)
  
  # set trajectory groundtruth and estimate file
  traj_groundtruth = params['dataset_directory']+'/groundtruth_filtered.txt'
  traj_estimate = results_dir+'/traj_estimate.txt'
  traj_estimate_rotated_filename = results_dir+'/traj_estimate_rotated.txt'
  translation_error_filename = results_dir+'/translation_error.txt'
  orientation_error_filename = results_dir+'/orientation_error.txt'
  scale_drift_filename = results_dir+'/scale_drift.txt'
  
  # load hand-eye calib: trafo between (m)arker and (c)amera.
  dataset_param_file = params['dataset_directory']+'/dataset_params.yaml'
  dataset_param_file_stream = open(dataset_param_file,'r')
  P = yaml.load(dataset_param_file_stream)
  T_cm_quat = np.array([P['calib_Tcm_qx'], P['calib_Tcm_qy'], P['calib_Tcm_qz'], P['calib_Tcm_qw']])
  T_cm_tran = np.array([P['calib_Tcm_tx'], P['calib_Tcm_ty'], P['calib_Tcm_tz']])
  T_cm = getRigidBodyTrafo(T_cm_quat, T_cm_tran)
  T_mc = transformations.inverse_matrix(T_cm)
  
  # load data
  data_gt = associate.read_file_list(traj_groundtruth)
  data_es = associate.read_file_list(traj_estimate)
  
  # find matches
  matches = associate.associate(data_gt, data_es, -P['cam_delay'], 0.02)
  p_gt = np.array([[float(value) for value in data_gt[a][0:3]] for a,b in matches])
  q_gt = np.array([[float(value) for value in data_gt[a][3:7]] for a,b in matches])
  p_es = np.array([[float(value) for value in data_es[b][0:3]] for a,b in matches])
  q_es = np.array([[float(value) for value in data_es[b][3:7]] for a,b in matches])
  t_gt = np.array([float(a) for a,b in matches])
  t_es = np.array([float(b) for a,b in matches])
  start_time = min(t_es[0], t_gt[0])
  t_es -= start_time
  t_gt -= start_time
  
  # align Sim3 to get scale
  if(n_align_frames > np.shape(matches)[0]):
    raise NameError('n_matches_for_scale is too large')
  scale,rot,trans = alignSim3(p_gt, p_es, n_align_frames)
  print 'scale = '+str(scale)
  
  # get trafo between (v)ision and (o)ptitrack frame
  T_om = getRigidBodyTrafo(q_gt[0,:], p_gt[0,:])
  T_vc = getRigidBodyTrafo(q_es[0,:], scale*p_es[0,:])
  T_cv = transformations.inverse_matrix(T_vc)
  T_ov = np.dot(T_om, np.dot(T_mc, T_cv))
  print 'T_ov = ' + str(T_ov)
  
  p_es_aligned = np.zeros(np.shape(p_es))
  q_es_aligned = np.zeros(np.shape(q_es))
  rpy_es_aligned = np.zeros(np.shape(p_es))
  rpy_gt = np.zeros(np.shape(p_es))
  
  # rotate trajectory
  for i in range(np.shape(matches)[0]):
    T_vc = getRigidBodyTrafo(q_es[i,:],p_es[i,:])
    T_vc[0:3,3] *= scale
    T_om = np.dot(T_ov, np.dot(T_vc, T_cm))
    p_es_aligned[i,:] = T_om[0:3,3]
    q_es_aligned[i,:] = transformations.quaternion_from_matrix(T_om)
    rpy_es_aligned[i,:] = transformations.euler_from_quaternion(q_es_aligned[i,:], 'rzyx')
    rpy_gt[i,:] = transformations.euler_from_quaternion(q_gt[i,:], 'rzyx')
  
  if save:
    f = open(traj_estimate_rotated_filename,'w')
    for i in range(np.shape(p_es_aligned)[0]):
      f.write('%.7f %.5f %.5f %.5f %.5f %.5f %.5f %.5f\n' % (matches[i][1]-P['cam_delay'], p_es_aligned[i,0], p_es_aligned[i,1], p_es_aligned[i,2], q_es_aligned[i,0], q_es_aligned[i,1], q_es_aligned[i,2], q_es_aligned[i,3]))
    f.close()
    
  # plot trajectory
  fig = plt.figure()
  ax = fig.add_subplot(111, title='trajectory', aspect='equal', xlabel='x [m]', ylabel='Y [m]')
  ax.plot(p_es_aligned[:,0], p_es_aligned[:,1], 'b-', label='estimate')
  ax.plot(p_gt[:,0], p_gt[:,1], 'r-', label='groundtruth')
  ax.plot(p_es_aligned[0:n_align_frames,0], p_es_aligned[0:n_align_frames,1], 'g-', linewidth=2, label='aligned')
  fig.tight_layout()
  fig.savefig(results_dir+'/trajectory.pdf')
          
  # plot position error (drift)
  translation_error = (p_gt-p_es_aligned)
  fig = plt.figure(figsize=(8, 2.5))
  ax = fig.add_subplot(111, xlabel='time [s]', ylabel='position drift [m]', xlim=[0,t_es[-1]+4])
  ax.plot(t_es, translation_error[:,0], 'r-', label='x')
  ax.plot(t_es, translation_error[:,1], 'g-', label='y')
  ax.plot(t_es, translation_error[:,2], 'b-', label='z')
  fig.tight_layout()
  
  if save:
    fig.savefig(results_dir+'/translation_error.pdf')
    f = open(translation_error_filename,'w')
    for i in range(translation_error.shape[0]):
      f.write('%.7f %.5f %.5f %.5f\n'%(t_es[i], translation_error[i,0],  translation_error[i,1],  translation_error[i,2]))
    f.close()
    
  # plot orientation error (drift)
  orientation_error = (rpy_gt - rpy_es_aligned)
  fig = plt.figure(figsize=(8, 2.5))
  ax = fig.add_subplot(111, xlabel='time [s]', ylabel='orientation drift [rad]', xlim=[0,t_es[-1]+4])
  ax.plot(t_es, orientation_error[:,0], 'r--', label='yaw')
  ax.plot(t_es, orientation_error[:,1], 'g--', label='pitch')
  ax.plot(t_es, orientation_error[:,2], 'b--', label='roll')
  ax.legend()
  fig.tight_layout()
  
  if save:
    fig.savefig(results_dir+'/orientation_error.pdf')
    f = open(orientation_error_filename,'w')
    for i in range(orientation_error.shape[0]):
      f.write('%.7f %.5f %.5f %.5f\n'%(t_es[i], orientation_error[i,0],  orientation_error[i,1],  orientation_error[i,2]))
    f.close()
    
  # plot scale drift
  motion_gt = np.diff(p_gt, 0)
  motion_es = np.diff(p_es_aligned, 0)
  dist_gt = np.sqrt(np.sum(np.multiply(motion_gt,motion_gt),1))
  dist_es = np.sqrt(np.sum(np.multiply(motion_es,motion_es),1))
  fig = plt.figure(figsize=(8,2.5))
  ax = fig.add_subplot(111, xlabel='time [s]', ylabel='scale change [\%]', xlim=[0,t_es[-1]+4])
  scale_drift = np.divide(dist_es,dist_gt)*100-100
  ax.plot(t_es, scale_drift, 'b-')
  fig.tight_layout()
  
  if save:
    fig.savefig(results_dir+'/scale_drift.pdf')
    f = open(scale_drift_filename,'w')
    for i in range(orientation_error.shape[0]):
      f.write('%.7f %.5f\n'%(t_es[i], scale_drift[i]))
    f.close()
    
