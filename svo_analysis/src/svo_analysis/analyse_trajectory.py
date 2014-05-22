#!/usr/bin/python

import os
import yaml
import argparse
import numpy as np
import matplotlib.pyplot as plt
import svo_analysis.tum_benchmark_tools.associate as associate
import vikit_py.transformations as transformations
import vikit_py.align_trajectory as align_trajectory
from matplotlib import rc
rc('font',**{'family':'serif','serif':['Cardo']})
rc('text', usetex=True)

def plot_translation_error(timestamps, translation_error, results_dir):
    fig = plt.figure(figsize=(8, 2.5))
    ax = fig.add_subplot(111, xlabel='time [s]', ylabel='position drift [mm]', xlim=[0,timestamps[-1]-timestamps[0]+4])
    ax.plot(timestamps-timestamps[0], translation_error[:,0]*1000, 'r-', label='x')
    ax.plot(timestamps-timestamps[0], translation_error[:,1]*1000, 'g-', label='y')
    ax.plot(timestamps-timestamps[0], translation_error[:,2]*1000, 'b-', label='z')
    ax.legend()
    fig.tight_layout()
    fig.savefig(results_dir+'/translation_error.pdf')

def plot_rotation_error(timestamps, rotation_error, results_dir):
    fig = plt.figure(figsize=(8, 2.5))
    ax = fig.add_subplot(111, xlabel='time [s]', ylabel='orientation drift [rad]', xlim=[0,timestamps[-1]-timestamps[0]+4])
    ax.plot(timestamps-timestamps[0], rotation_error[:,0], 'r-', label='yaw')
    ax.plot(timestamps-timestamps[0], rotation_error[:,1], 'g-', label='pitch')
    ax.plot(timestamps-timestamps[0], rotation_error[:,2], 'b-', label='roll')
    ax.legend()
    fig.tight_layout()
    fig.savefig(results_dir+'/orientation_error.pdf')

def analyse_synthetic_trajectory(results_dir):
    data = np.loadtxt(os.path.join(results_dir, 'translation_error.txt'))
    timestamps = data[:,0]
    translation_error = data[:,1:4]
    plot_translation_error(timestamps, translation_error, results_dir)
      
    # plot orientation error
    data = np.loadtxt(os.path.join(results_dir, 'orientation_error.txt'))
    timestamps = data[:,0]
    orientation_error = data[:,1:4]
    plot_rotation_error(timestamps, orientation_error, results_dir)
    
def analyse_optitrack_trajectory_with_hand_eye_calib(results_dir, params, n_align_frames = 200):
    print('loading hand-eye-calib')
    T_cm_quat = np.array([params['hand_eye_calib']['Tcm_qx'],
                          params['hand_eye_calib']['Tcm_qy'],
                          params['hand_eye_calib']['Tcm_qz'],
                          params['hand_eye_calib']['Tcm_qw']])
    T_cm_tran = np.array([params['hand_eye_calib']['Tcm_tx'],
                          params['hand_eye_calib']['Tcm_ty'],
                          params['hand_eye_calib']['Tcm_tz']])
    T_cm = get_rigid_body_trafo(T_cm_quat, T_cm_tran)
    T_mc = transformations.inverse_matrix(T_cm)

    t_es, p_es, q_es, t_gt, p_gt, q_gt = load_dataset(results_dir, params['cam_delay'])
    
    # align Sim3 to get scale
    print('align Sim3 using '+str(n_align_frames)+' first frames.')
    scale,rot,trans = align_trajectory.align_sim3(p_gt[0:n_align_frames,:], p_es[0:n_align_frames,:])
    print 'scale = '+str(scale)
  
    # get trafo between (v)ision and (o)ptitrack frame
    print q_gt[0,:]
    print p_gt[0,:]
    T_om = get_rigid_body_trafo(q_gt[0,:], p_gt[0,:])
    T_vc = get_rigid_body_trafo(q_es[0,:], scale*p_es[0,:])
    T_cv = transformations.inverse_matrix(T_vc)
    T_ov = np.dot(T_om, np.dot(T_mc, T_cv))
    print 'T_ov = ' + str(T_ov)
  
    # apply transformation to estimated trajectory
    q_es_aligned = np.zeros(np.shape(q_es))
    rpy_es_aligned = np.zeros(np.shape(p_es))
    rpy_gt = np.zeros(np.shape(p_es))
    p_es_aligned = np.zeros(np.shape(p_es))
    for i in range(np.shape(p_es)[0]):
        T_vc = get_rigid_body_trafo(q_es[i,:],p_es[i,:])
        T_vc[0:3,3] *= scale
        T_om = np.dot(T_ov, np.dot(T_vc, T_cm))
        p_es_aligned[i,:] = T_om[0:3,3]
        q_es_aligned[i,:] = transformations.quaternion_from_matrix(T_om)
        rpy_es_aligned[i,:] = transformations.euler_from_quaternion(q_es_aligned[i,:], 'rzyx')
        rpy_gt[i,:] = transformations.euler_from_quaternion(q_gt[i,:], 'rzyx') 
  
    # plot position error (drift)
    translation_error = (p_gt-p_es_aligned)
    plot_translation_error(t_es, translation_error, results_dir)
  
    # plot orientation error (drift)
    orientation_error = (rpy_gt - rpy_es_aligned)
    plot_rotation_error(t_es, orientation_error, results_dir)

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
    fig.savefig(results_dir+'/scale_drift.pdf')
    
    # plot trajectory
    fig = plt.figure()
    ax = fig.add_subplot(111, title='trajectory', aspect='equal', xlabel='x [m]', ylabel='y [m]')
    ax.plot(p_es_aligned[:,0], p_es_aligned[:,1], 'b-', label='estimate')
    ax.plot(p_gt[:,0], p_gt[:,1], 'r-', label='groundtruth')
    ax.plot(p_es_aligned[0:n_align_frames,0], p_es_aligned[0:n_align_frames,1], 'g-', linewidth=2, label='aligned')
    ax.legend()
    fig.tight_layout()
    fig.savefig(results_dir+'/trajectory.pdf')
    
def analyse_trajectory(results_dir, n_align_frames = 200, use_hand_eye_calib = True):
    params = yaml.load(open(os.path.join(results_dir, 'dataset_params.yaml'),'r'))
    if params['dataset_is_blender']:
        analyse_synthetic_trajectory(results_dir)
    elif use_hand_eye_calib:
        analyse_optitrack_trajectory_with_hand_eye_calib(results_dir, params, n_align_frames)
    else:
        t_es, p_es, q_es, t_gt, p_gt, q_gt = load_dataset(results_dir, params['cam_delay'])
        scale,rot,trans = align_trajectory.align_sim3(p_gt[0:n_align_frames,:], p_es[0:n_align_frames,:])
        p_es_aligned = np.zeros(np.shape(p_es))
        for i in range(np.shape(p_es)[0]):
            p_es_aligned[i,:] = scale*rot.dot(p_es[i,:]) + trans
        # plot position error (drift)
        translation_error = (p_gt-p_es_aligned)
        plot_translation_error(t_es, translation_error, results_dir)
        
def get_rigid_body_trafo(quat,trans):
    T = transformations.quaternion_matrix(quat)
    T[0:3,3] = trans
    return T
     
def load_dataset(results_dir, cam_delay):
    print('loading dataset in '+results_dir)   
    print('cam_delay = '+str(cam_delay))

    data_gt = open(os.path.join(results_dir, 'groundtruth.txt')).read()
    lines = data_gt.replace(","," ").replace("\t"," ").split("\n") 
    data_gt = np.array([[np.float(v.strip()) for i,v in enumerate(line.split(" ")) if v.strip()!=""] for line in lines if len(line)>0 and line[0]!="#"])
    #data_gt = np.array([[np.float(v.strip()) for i,v in enumerate(line.split(" ")) if i != 1 and v.strip()!=""] for line in lines if len(line)>0 and line[0]!="#"])
    data_gt = [(float(l[0]),l[1:]) for l in data_gt]
    data_gt = dict(data_gt)    
    
    data_es = open(os.path.join(results_dir, 'traj_estimate.txt')).read()
    lines = data_es.replace(","," ").replace("\t"," ").split("\n") 
    data_es = np.array([[np.float(v.strip()) for v in line.split(" ") if v.strip()!=""] for line in lines if len(line)>0 and line[0]!="#"])
    data_es = [(float(l[0]),l[1:]) for l in data_es]
    data_es = dict(data_es)
    
    matches = associate.associate(data_gt, data_es, -cam_delay, 0.02)
    p_gt = np.array([[np.float(value) for value in data_gt[a][0:3]] for a,b in matches])
    q_gt = np.array([[np.float(value) for value in data_gt[a][3:7]] for a,b in matches])
    p_es = np.array([[np.float(value) for value in data_es[b][0:3]] for a,b in matches])
    q_es = np.array([[np.float(value) for value in data_es[b][3:7]] for a,b in matches])
    t_gt = np.array([np.float(a) for a,b in matches])
    t_es = np.array([np.float(b) for a,b in matches])
  
    # set start time to zero
    start_time = min(t_es[0], t_gt[0])
    t_es -= start_time
    t_gt -= start_time
    return t_es, p_es, q_es, t_gt, p_gt, q_gt
  
if __name__ == '__main__':
  
    # parse command line
    parser = argparse.ArgumentParser(description='''
    Analyse trajectory
    ''')
    parser.add_argument('results_dir', help='folder with the results')
    parser.add_argument('--use_hand_eye_calib', help='', action='store_true')
    parser.add_argument('--n_align_frames', help='', default=200)
    args = parser.parse_args()
    
    print('analyse trajectory for dataset: '+str(args.results_dir))
    analyse_trajectory(args.results_dir,
                       n_align_frames = int(args.n_align_frames),
                       use_hand_eye_calib = args.use_hand_eye_calib)