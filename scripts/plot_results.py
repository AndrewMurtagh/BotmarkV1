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

labels = ['NVIDIA', 'NUC', 'I9', 'XPS']
colors = ['lightgreen', 'lightgreen', 'lightgreen', 'lightgreen']

#hz
nvidia_per_trck = []
nuc_per_trck = []
i9_per_trck = []
xps_per_trck = []

#hz
nvidia_face_rec = []
nuc_face_rec = []
i9_face_rec = []
xps_face_rec = []

#hz
nvidia_slam = []
nuc_slam = []
i9_slam = []
xps_slam = []

#ms
nvidia_path_pln = []
nuc_path_pln = []
i9_path_pln = []
xps_path_pln = []

#hz
nvidia_obj_rec = []
nuc_obj_rec = []
i9_obj_rec = []
xps_obj_rec = []

#ms
nvidia_voi_rec = []
nuc_voi_rec = []
i9_voi_rec = []
xps_voi_rec = []

#ms
nvidia_spc_syn = []
nuc_spc_syn = []
i9_spc_syn = []
xps_spc_syn = []


per_trck_data = [nvidia_per_trck,nuc_per_trck,i9_per_trck,xps_per_trck]
face_rec_data = [nvidia_face_rec,nuc_face_rec,i9_face_rec,xps_face_rec]
slam_data = [nvidia_slam,nuc_slam,i9_slam,xps_slam]
path_pln_data = [nvidia_path_pln,nuc_path_pln,i9_path_pln,xps_path_pln]
obj_rec_data = [nvidia_obj_rec,nuc_obj_rec,i9_obj_rec,xps_obj_rec]
voi_rec_data = [nvidia_voi_rec,nuc_voi_rec,i9_voi_rec,xps_voi_rec]
spc_syn_data = [nvidia_spc_syn,nuc_spc_syn,i9_spc_syn,xps_spc_syn]

fig1, ax1 = plt.subplots()
box = ax1.boxplot(slam_data, patch_artist=True, labels=labels)
plt.ylim(ymin=0,ymax=25)
plt.ylabel('Readings rate [hz]')
for patch, color in zip(box['boxes'], colors):
    patch.set_facecolor(color)
for median in box['medians']:
    median.set(color='k', linewidth=1.5,)
plt.show()
