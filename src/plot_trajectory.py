#python3 

import pandas as pd
import csv
import matplotlib.pyplot as plt
df = pd.read_csv('/home/daun/.ros/scout.csv')
print (df)

x = df['estimate_p_x']
y = df['estimate_p_y']

# plot
fig, axs = plt.subplots(4, 3)

axs[0, 0].plot(df['estimate_p_x'],df['estimate_p_y'])
axs[0, 0].axis('equal')
axs[0, 1].plot(df['t'],df['steady_state'])
axs[0, 0].set_title('XY')

axs[1, 0].plot(df['t'],df['estimate_p_x'])
axs[1, 1].plot(df['t'],df['estimate_p_y'])
axs[1, 2].plot(df['t'],df['estimate_p_z'])
axs[1, 0].set_title('X')
axs[1, 1].set_title('Y')
axs[1, 2].set_title('Z')

axs[2, 0].plot(df['t'],df['odom_p_x'])
axs[2, 1].plot(df['t'],df['odom_p_y'])
axs[2, 2].plot(df['t'],df['odom_p_z'])
axs[2, 0].set_title('Odom X')
axs[2, 1].set_title('Odom Y')
axs[2, 2].set_title('Odom Z')

axs[3, 0].plot(df['t'],df['estimate_roll'])
axs[3, 1].plot(df['t'],df['estimate_pitch'])
axs[3, 2].plot(df['t'],df['estimate_yaw'])
axs[3, 0].set_title('Odom roll')
axs[3, 1].set_title('Odom pitch')
axs[3, 2].set_title('Odom yaw')

# beautify the x-labels
# plt.gcf().autofmt_xdate()

plt.show()