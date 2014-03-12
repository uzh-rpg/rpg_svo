#!/usr/bin/python

import yaml
import argparse
import random
import numpy
import sys
import transformations
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
from matplotlib import rc

def transform44(l):
    t = l[1:4]
    q = l[4:8]
    return numpy.dot(transformations.translation_matrix(t),transformations.quaternion_matrix(q))

def read_trajectory(filename, matrix=True):
    file = open(filename)
    data = file.read()
    lines = data.replace(","," ").replace("\t"," ").split("\n")
    list = [[float(v.strip()) for v in line.split(" ") if v.strip()!=""] for line in lines if len(line)>0 and line[0]!="#"]
    list_ok = []
    for i,l in enumerate(list):
        if l[4:8]==[0,0,0,0]:
            continue
        isnan = False
        for v in l:
            if numpy.isnan(v):
                isnan = True
                break
        if isnan:
            sys.stderr.write("Warning: line %d of file '%s' has NaNs, skipping line\n"%(i,filename))
            continue
        list_ok.append(l)
    if matrix :
      traj = dict([(l[0],transform44(l[0:])) for l in list_ok])
    else:
      traj = dict([(l[0],l[1:8]) for l in list_ok])
    return traj

def find_closest_index(L,t):
    beginning = 0
    difference = abs(L[0] - t)
    best = 0
    end = len(L)
    while beginning < end:
        middle = int((end+beginning)/2)
        if abs(L[middle] - t) < difference:
            difference = abs(L[middle] - t)
            best = middle
        if t == L[middle]:
            return middle
        elif L[middle] > t:
            end = middle
        else:
            beginning = middle + 1
    return best

def ominus(a,b):
    return numpy.dot(numpy.linalg.inv(a),b)

def scale(a,scalar):
    return numpy.array(
        [[a[0,0], a[0,1], a[0,2], a[0,3]*scalar],
         [a[1,0], a[1,1], a[1,2], a[1,3]*scalar],
         [a[2,0], a[2,1], a[2,2], a[2,3]*scalar],
         [a[3,0], a[3,1], a[3,2], a[3,3]]]
                       )

def compute_distance(transform):
    return numpy.linalg.norm(transform[0:3,3])

def compute_angle(transform):
    # an invitation to 3-d vision, p 27
    return numpy.arccos( min(1,max(-1, (numpy.trace(transform[0:3,0:3]) - 1)/2) ))

def distances_along_trajectory(traj):
    keys = traj.keys()
    keys.sort()
    motion = [ominus(traj[keys[i+1]],traj[keys[i]]) for i in range(len(keys)-1)]
    distances = [0]
    sum = 0
    for t in motion:
        sum += compute_distance(t)
        distances.append(sum)
    return distances

def rotations_along_trajectory(traj,scale):
    keys = traj.keys()
    keys.sort()
    motion = [ominus(traj[keys[i+1]],traj[keys[i]]) for i in range(len(keys)-1)]
    distances = [0]
    sum = 0
    for t in motion:
        sum += compute_angle(t)*scale
        distances.append(sum)
    return distances

def interpolate_transformation(alpha,a,b):
    qa,ta = transformations.quaternion_from_matrix(a),transformations.translation_from_matrix(a)
    qb,tb = transformations.quaternion_from_matrix(b),transformations.translation_from_matrix(b)
    qr = transformations.quaternion_slerp(qa,qb,alpha)
    tr = (1-alpha) * ta + (alpha) * tb
    r = numpy.dot(transformations.translation_matrix(tr),transformations.quaternion_matrix(qr))
    return r

def interpolate_and_resample(traj_gt,stamps_est,param_offset,gt_max_time_difference):
    traj_new = {}

    stamps_gt = list(traj_gt.keys())
    stamps_gt.sort()

    for t_est in stamps_est:
        t_new = t_est + param_offset
        i_nearest = find_closest_index(stamps_gt,t_est + param_offset)
        if stamps_gt[i_nearest] >= t_new and i_nearest>0:
            i_low = i_nearest-1
            i_high = i_nearest
        else:
            i_low = i_nearest
            i_high = i_nearest+1
        if i_low<0 or i_high>=len(stamps_gt):
            continue

        t_low = stamps_gt[i_low]
        t_high = stamps_gt[i_high]

        if(abs( t_new - (t_low + param_offset) ) > gt_max_time_difference or
           abs( t_new - (t_high + param_offset) ) > gt_max_time_difference):
            continue

        alpha = (t_new - (t_low + param_offset)) / (t_high - t_low)
        transform = interpolate_transformation(alpha,traj_gt[t_low],traj_gt[t_high])
        traj_new[t_new] = transform
    return traj_new


def evaluate_trajectory(traj_gt,traj_est,param_max_pairs=10000,param_fixed_delta=False,param_delta=1.00,param_delta_unit="s",param_offset=0.00,param_scale=1.00,param_interpolation=True):
    stamps_est = list(traj_est.keys())
    stamps_est.sort()
    stamps_gt = list(traj_gt.keys())
    stamps_gt.sort()

    gt_interval = numpy.median([s-t for s,t in zip(stamps_gt[1:],stamps_gt[:-1])])
    gt_max_time_difference = 2*gt_interval

    if param_interpolation:
        traj_gt = interpolate_and_resample(traj_gt,stamps_est,param_offset,gt_max_time_difference)
        stamps_gt = list(traj_gt.keys())
        stamps_gt.sort()

    stamps_est_return = []
    for t_est in stamps_est:
        t_gt = stamps_gt[find_closest_index(stamps_gt,t_est + param_offset)]
        t_est_return = stamps_est[find_closest_index(stamps_est,t_gt - param_offset)]
        t_gt_return = stamps_gt[find_closest_index(stamps_gt,t_est_return + param_offset)]
        if not t_est_return in stamps_est_return:
            stamps_est_return.append(t_est_return)
    if(len(stamps_est_return)<2):
        raise Exception("Number of overlap in the timestamps is too small. Did you run the evaluation on the right files?")

    if param_delta_unit=="s":
        index_est = list(traj_est.keys())
        index_est.sort()
    elif param_delta_unit=="m":
        index_est = distances_along_trajectory(traj_est)
    elif param_delta_unit=="rad":
        index_est = rotations_along_trajectory(traj_est,1)
    elif param_delta_unit=="deg":
        index_est = rotations_along_trajectory(traj_est,180/numpy.pi)
    elif param_delta_unit=="f":
        index_est = range(len(traj_est))
    else:
        raise Exception("Unknown unit for delta: '%s'"%param_delta_unit)

    if not param_fixed_delta:
        if(param_max_pairs==0 or len(traj_est)<numpy.sqrt(param_max_pairs)):
            pairs = [(i,j) for i in range(len(traj_est)) for j in range(len(traj_est))]
        else:
            pairs = [(random.randint(0,len(traj_est)-1),random.randint(0,len(traj_est)-1)) for i in range(param_max_pairs)]
    else:
        pairs = []
        for i in range(len(traj_est)):
            j = find_closest_index(index_est,index_est[i] + param_delta)
            if j!=len(traj_est)-1:
                pairs.append((i,j))
        if(param_max_pairs!=0 and len(pairs)>param_max_pairs):
            pairs = random.sample(pairs,param_max_pairs)

    result = []
    for i,j in pairs:
        stamp_est_0 = stamps_est[i]
        stamp_est_1 = stamps_est[j]

        stamp_gt_0 = stamps_gt[ find_closest_index(stamps_gt,stamp_est_0 + param_offset) ]
        stamp_gt_1 = stamps_gt[ find_closest_index(stamps_gt,stamp_est_1 + param_offset) ]

        if(abs(stamp_gt_0 - (stamp_est_0 + param_offset)) > gt_max_time_difference  or
           abs(stamp_gt_1 - (stamp_est_1 + param_offset)) > gt_max_time_difference):
            continue

        error44 = ominus(  scale(
                           ominus( traj_est[stamp_est_1], traj_est[stamp_est_0] ),param_scale),
                           ominus( traj_gt[stamp_gt_1], traj_gt[stamp_gt_0] ) )

        
        trans = compute_distance(error44)
        rot = compute_angle(error44)

        motion_gt = ominus( traj_gt[stamp_gt_1], traj_gt[stamp_gt_0] )
        trans_gt = compute_distance(motion_gt)
        rot_gt = compute_angle(motion_gt)
        
        if(rot < 0.07):
          result.append([stamp_est_0,stamp_est_1,stamp_gt_0,stamp_gt_1,trans,rot,trans_gt,rot_gt])

    if len(result)<2:
        raise Exception("Couldn't find matching timestamp pairs between groundtruth and estimated trajectory!")

    return result

def percentile(seq,q):
    seq_sorted = list(seq)
    seq_sorted.sort()
    return seq_sorted[int((len(seq_sorted)-1)*q)]

if __name__ == '__main__':
    random.seed(0)

    # config -------------------------------------------------------------------------
    
    dataset='20131003_1937_nslam_i7_asl2_fast'
    dataset_dir = '../results/'+dataset
    
    #---------------------------------------------------------------------------------
    
    # load dataset parameters
    params_stream = open(dataset_dir+'/params.yaml')
    params = yaml.load(params_stream)
    
    # set trajectory groundtruth and estimate file
    traj_groundtruth = params['dataset_directory']+'/groundtruth_filtered.txt'
    traj_estimate_rotated = dataset_dir+'/traj_estimate_rotated.txt'

    parser = argparse.ArgumentParser(description='''
    This script computes the relative pose error from the ground truth trajectory and the estimated trajectory.
    ''')
    parser.add_argument('--groundtruth_file', help='ground-truth trajectory file (format: "timestamp tx ty tz qx qy qz qw")', default=traj_groundtruth)
    parser.add_argument('--estimated_file', help='estimated trajectory file (format: "timestamp tx ty tz qx qy qz qw")', default=traj_estimate_rotated)
    parser.add_argument('--interpolate', help='interpolate and resample the ground truth trajectory', default=True)
    parser.add_argument('--max_pairs', help='maximum number of pose comparisons (default: 10000, set to zero to disable downsampling)', default=10000)
    parser.add_argument('--fixed_delta', help='only consider pose pairs that have a distance of delta delta_unit (e.g., for evaluating the drift per second/meter/radian)', default=True)
    parser.add_argument('--delta', help='delta for evaluation (default: 1.0)',default=1.0)
    parser.add_argument('--delta_unit', help='unit of delta (options: \'s\' for seconds, \'m\' for meters, \'rad\' for radians, \'f\' for frames; default: \'s\')',default='s')
    parser.add_argument('--offset', help='time offset between ground-truth and estimated trajectory (default: 0.0)',default=0.0)
    parser.add_argument('--scale', help='scaling factor for the estimated trajectory (default: 1.0)',default=1.0)
    parser.add_argument('--save', help='text file to which the evaluation will be saved (format: stamp_est0 stamp_est1 stamp_gt0 stamp_gt1 trans_error rot_error)')
    parser.add_argument('--plot', help='plot the result to a file (requires --fixed_delta, output format: png)', default=True)
    parser.add_argument('--plotname', help='plot the result to a file (requires --fixed_delta, output format: png)', default='rpe.png')
    parser.add_argument('--verbose', help='print all evaluation data (otherwise, only the mean translational error measured in meters will be printed)', default=True)
    args = parser.parse_args()

    if args.plot and not args.fixed_delta:
        sys.exit("The '--plot' option can only be used in combination with '--fixed_delta'")

    traj_gt = read_trajectory(args.groundtruth_file)
    traj_est = read_trajectory(args.estimated_file)

    result = evaluate_trajectory(traj_gt,
                                 traj_est,
                                 int(args.max_pairs),
                                 args.fixed_delta,
                                 float(args.delta),
                                 args.delta_unit,
                                 float(args.offset),
                                 float(args.scale),
                                 args.interpolate)

    stamps = numpy.array(result)[:,0]
    trans_error = numpy.array(result)[:,4]
    rot_error = numpy.array(result)[:,5]
    
    
    results = dict()
    results['trans_rmse'] = float(numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error)))
    results['trans_mean'] = float(numpy.mean(trans_error))
    results['trans_median'] = float(numpy.median(trans_error))
    results['trans_std'] = float(numpy.std(trans_error))
    results['trans_min'] = float(numpy.min(trans_error))
    results['trans_max'] = float(numpy.max(trans_error))
    results['rot_rmse'] = float((numpy.sqrt(numpy.dot(rot_error,rot_error) / len(rot_error)) * 180.0 / numpy.pi))
    results['rot_mean'] = float((numpy.mean(rot_error) * 180.0 / numpy.pi))
    results['rot_median'] = float(numpy.median(rot_error) * 180.0 / numpy.pi)
    results['rot_std'] = float((numpy.std(rot_error) * 180.0 / numpy.pi))
    results['rot_min'] = float((numpy.min(rot_error) * 180.0 / numpy.pi))
    results['rot_max'] = float((numpy.max(rot_error) * 180.0 / numpy.pi))
    
    # dump experiment results to file:
    results_file = params['trace_dir']+'/rpe_results.yaml'
    with open(results_file,'w') as outfile:
      outfile.write( yaml.dump(results, default_flow_style=False) )
    
    if args.save: 
        f = open(args.save,"w")
        f.write("\n".join([" ".join(["%f"%v for v in line]) for line in result]))
        f.close()

    if args.verbose:
        print "compared_pose_pairs %d pairs"%(len(trans_error))

        print "translational_error.rmse %f m"%results['trans_rmse']
        print "translational_error.mean %f m"%results['trans_mean']
        print "translational_error.median %f m"%results['trans_median']
        print "translational_error.std %f m"%results['trans_std']
        print "translational_error.min %f m"%results['trans_min']
        print "translational_error.max %f m"%results['trans_max']

        print "rotational_error.rmse %f deg"%results['rot_rmse']
        print "rotational_error.mean %f deg"%results['rot_mean']
        print "rotational_error.median %f deg"%results['rot_median']
        print "rotational_error.std %f deg"%results['rot_std']
        print "rotational_error.min %f deg"%results['rot_min']
        print "rotational_error.max %f deg"%results['rot_max']
    else:
        print numpy.mean(trans_error)

    if args.plot:
      
        # tell matplotlib to use latex font
        rc('font',**{'family':'serif','serif':['Cardo']})
        rc('text', usetex=True)
  
        #matplotlib.use('Agg') # no window should appear?

        fig = plt.figure(figsize=(8, 3))
        ax = fig.add_subplot(111)
        ax.plot(stamps - stamps[0],trans_error,'-',color="blue")
        #ax.plot([t for t,e in err_rot],[e for t,e in err_rot],'-',color="red")
        ax.set_xlabel('time [s]')
        ax.set_ylabel('translational error [m]')
        plt.tight_layout()
        plt.savefig(dataset_dir+'/rpe_trans_error.pdf')
        plt.show()

        fig = plt.figure(figsize=(8, 3))
        ax = fig.add_subplot(111)
        ax.plot(stamps - stamps[0],rot_error* 180.0 / numpy.pi,'-',color="blue")
        #ax.plot([t for t,e in err_rot],[e for t,e in err_rot],'-',color="red")
        ax.set_xlabel('time [s]')
        ax.set_ylabel('rot error [deg]')
        plt.tight_layout()
        plt.savefig(dataset_dir+'/rpe_rot_error.pdf')
        plt.show()