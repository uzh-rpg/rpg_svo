#!/usr/bin/python


import numpy as np
import matplotlib.pyplot as plt
import transformations
from scipy import signal

save = True
data_filename = '/home/cforster/Datasets/SlamBenchmark/asl_vicon_d2/groundtruth.txt'
filtered_data_filename = '/home/cforster/Datasets/SlamBenchmark/asl_vicon_d2/groundtruth_filtered.txt'
file = open(data_filename)
data = file.read()
lines = data.replace(","," ").replace("\t"," ").split("\n")
D = np.array([[v.strip() for v in line.split(" ") if v.strip()!=""] for line in lines if len(line)>0 and line[0]!="#"], dtype=np.float64)

n = np.shape(D)[0]
rpy = np.empty([n,3])
for i in range(n):
  quat = D[i,4:8]
  rpy[i,:] = transformations.euler_from_quaternion(quat, axes='sxyz')

# filter rpy
f_sensor = 200.0; # sampling frequency in hz
f_cut = 15.0; # cutoff frequency in hz
b,a = signal.butter(5,f_cut/(f_sensor/2));
print a
print b

rpy_filt = np.empty([n,3])
rpy_filt[:,0] = signal.filtfilt(b, a, rpy[:,0])
rpy_filt[:,1] = signal.filtfilt(b, a, rpy[:,1])
rpy_filt[:,2] = signal.filtfilt(b, a, rpy[:,2])

fig = plt.figure()
ax = fig.add_subplot(111, title='orientation filtered')
ax.plot(rpy[:,0], 'r-')
ax.plot(rpy[:,1], 'g-')
ax.plot(rpy[:,2], 'b-')
ax.plot(rpy_filt[:,0], 'k-', linewidth=2)
ax.plot(rpy_filt[:,1], 'k-', linewidth=2)
ax.plot(rpy_filt[:,2], 'k-', linewidth=2)

fig = plt.figure()
ax = fig.add_subplot(111, title='position')
ax.plot(D[:,1], 'r')
ax.plot(D[:,2], 'g')
ax.plot(D[:,3], 'b')

fig = plt.figure()
ax = fig.add_subplot(111, title='trajectory from top')
ax.plot(D[:,1], D[:,2])

if save:
  f = open(filtered_data_filename,'w')
  for i in range(np.shape(D)[0]):
    quat = transformations.quaternion_from_euler(rpy_filt[i,0], rpy_filt[i,1], rpy_filt[i,2], axes='sxyz')
    f.write('%.7f %.5f %.5f %.5f %.5f %.5f %.5f %.5f\n' % (D[i,0], D[i,1], D[i,2], D[i,3], quat[0], quat[1], quat[2], quat[3]))
  
  f.close()
