"""
This file is part of the Botmark benchmark.

Copyright (c) 2019 Andrew Murtagh, Patrick Lynch,
and Conor McGinn.

This work is licensed under the "Creative Commons
(Attribution-NonCommercial-ShareAlike 4.0 International)
License" and is copyrighted by Andrew Murtagh, Patrick Lynch,
and Conor McGinn.

To view a copy of this license, visit
http://creativecommons.org/licenses/by-nc-sa/4.0/ or
send a letter to Creative Commons, PO Box 1866,
Mountain View, CA 94042, USA.

Botmark is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.

"""

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


#hz
native_per_trck_mean = 0
container_per_trc_mean = 0
vm_per_trck = 0
pertrc = [native_per_trck_mean, container_per_trc_mean, vm_per_trck]


#hz
native_face_rec_mean = 0
container_face_rec_mean = 0
vm_face_rec = 0
face = [native_face_rec_mean, container_face_rec_mean, vm_face_rec]

#hz
native_slam_mean = 0
container_slam_mean = 0
vm_slam_mean = 0
slam = [native_slam_mean, container_slam_mean, vm_slam_mean]

#ms
native_path_pln_mean = 0
container_path_pln_mean = 0
vm_path_pln_mean = 0
path = [native_path_pln_mean, container_path_pln_mean, vm_path_pln_mean]

#hz
native_obj_rec_mean = 0
vm_obj_rec = 0
container_obj_rec_mean = 0
obj = [native_obj_rec_mean, container_obj_rec_mean, vm_obj_rec]


#ms
native_voi_rec_mean = 0
container_voi_rec_mean = 0
vm_voi_rec_mean = 0
voice = [native_voi_rec_mean, container_voi_rec_mean, vm_voi_rec_mean]

#ms
native_spc_syn_mean = 0
container_spc_syn_mean = 0
vm_spc_syn = 0
speech = [native_spc_syn_mean, container_spc_syn_mean, vm_spc_syn]


fig1, ax1 = plt.subplots()
colors = ['lightgreen','lightblue','lightcoral']
labels = ['Native','Docker','VM']
bar = plt.bar([1,2,3], obj, color=colors, linewidth=0.5, edgecolor=['black']*3)
ax1.set_xticks([1,2,3])
ax1.set_xticklabels(labels)
for tic in ax1.xaxis.get_major_ticks():
    tic.tick1On = tic.tick2On = False
plt.ylabel('Frame rate [hz]')
# plt.ylabel('Readings rate [hz]')
# plt.ylabel('Execution time [ms]')

plt.show()
