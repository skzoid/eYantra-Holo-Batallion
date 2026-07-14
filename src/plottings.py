#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import pickle

with open("yaw_data.pkl","rb") as f:
    yaw_data = pickle.load(f)

# Convert yaw_data to numpy arrays for plotting
yaw_data_arrays = {id: np.array(yaws) for id, yaws in yaw_data.items()}
yaw_data_filtered_arrays ={}
yaw_data_running_avg = {}
yaw_prev = None
running_avg = 0
count = 0
for ids, yaws in yaw_data_arrays.items():
    yaw_data_filtered_arrays[ids] = []
    yaw_data_running_avg[ids] = []
    for yaw in yaws:
        if yaw_prev is None:
            yaw_prev = yaw
            yaw_data_filtered_arrays[ids].append(yaw)
            continue

        if abs(yaw - yaw_prev) > 3:
            # reject jump → repeat previous value
            yaw_data_filtered_arrays[ids].append(yaw_prev)
        else:
            yaw_data_filtered_arrays[ids].append(yaw)
            yaw_prev = yaw
        count += 1
        running_avg = (running_avg * (count-1) + yaw) / count 
        yaw_data_running_avg[ids].append(running_avg)
    running_avg = 0
    count = 0
    yaw_prev =None


# Plot
# for i, (ids, yaws) in enumerate(yaw_data_arrays.items()):
#     fig_1,axs_1 = plt.subplots(3,1,figsize=(8,8))
#     axs_1[0].plot(yaws)
#     axs_1[1].plot(np.array(yaw_data_filtered_arrays[ids]), label='Filtered', color='orange')
#     axs_1[0].set_title(f"Unfiltered,{ids}")
#     axs_1[1].set_title(f"filtered")
#     axs_1[2].plot(yaw_data_running_avg[ids], label='avg', color='black')
#     if ids == 21:
#         y = [6.004720389910939 * 180 / np.pi] * len(yaw_data_running_avg[ids])
#         x = np.arange(len(yaw_data_running_avg[ids]))
#         axs_1[2].plot(y , color = 'red')
#     if ids == 30:
#         y = [4.851063506770642 * 180 / np.pi] * len(yaw_data_running_avg[ids])
#         x = np.arange(len(yaw_data_running_avg[ids]))
#         axs_1[2].plot(y , color = 'red')
#     if ids == 13:
#         y = [3.1088203258474962 * 180 / np.pi] * len(yaw_data_running_avg[ids])
#         x = np.arange(len(yaw_data_running_avg[ids]))
#         axs_1[2].plot(y , color = 'red')


fig,axs =  plt.subplots(5,3,figsize=(8,8))
for i, (ids, yaws) in enumerate(yaw_data_arrays.items()):
    
    axs[i][0].plot(yaws)
    axs[i][1].plot(np.array(yaw_data_filtered_arrays[ids]), label='Filtered', color='orange')
    axs[i][0].set_title(f"Unfiltered,{ids}")
    axs[i][1].set_title(f"filtered")
    axs[i][2].plot(yaw_data_running_avg[ids], label='avg', color='black')
    if ids == 21:
        y = [1.5620574299939105 * 180 / np.pi] * len(yaw_data_running_avg[ids])
        x = np.arange(len(yaw_data_running_avg[ids]))
        
    if ids == 30:
        y = [3.308792564727693 * 180 / np.pi] * len(yaw_data_running_avg[ids])
        x = np.arange(len(yaw_data_running_avg[ids]))
        
    if ids == 13:
        y = [4.944075473106095 * 180 / np.pi] * len(yaw_data_running_avg[ids])
        x = np.arange(len(yaw_data_running_avg[ids]))
        
    if ids == 16:
        y = [4.6695983732537325 * 180 / np.pi] * len(yaw_data_running_avg[ids])
        x = np.arange(len(yaw_data_running_avg[ids]))
        
    if ids == 12:
        y = [3.126429413291209 * 180 / np.pi] * len(yaw_data_running_avg[ids])
        x = np.arange(len(yaw_data_running_avg[ids]))

    axs[i][2].plot(y , color = 'red')
    axs[i][2].set_ylim((y[0] - 5,y[0]+5))
    
    
plt.tight_layout()
plt.show()
