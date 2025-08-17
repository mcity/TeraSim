import matplotlib
import matplotlib.animation as animation
import matplotlib.pyplot as plt

import SumoTrajVis
matplotlib.use('Agg')
# Load net file and trajectory file
net = SumoTrajVis.Net("/home/mtl/cosmos-av-sample-toolkits/terasim_dataset/e7078100-3635-4e58-a497-64e5528f08e8/map.net.xml")
trajectories = SumoTrajVis.Trajectories("/home/mtl/cosmos-av-sample-toolkits/terasim_dataset/e7078100-3635-4e58-a497-64e5528f08e8/fcd_all.xml")
# Set trajectory color for different vehicles
for trajectory in trajectories:
    if trajectory.id == "CAV":
        trajectory.assign_colors_constant("#ff0000")
    else:
        trajectory.assign_colors_constant("#00FF00")

# Show the generated trajectory video
fig, ax = plt.subplots()
ax.set_aspect('equal', adjustable='box') # same scale
artist_collection = net.plot(ax=ax)
plot_time_interaval = trajectories.timestep_range()[-100:] # only plot 10s before the end of the trajectories, can be modified later
a = animation.FuncAnimation(
    fig,
    trajectories.plot_points,
    frames=plot_time_interaval,
    interval=1,
    fargs=(ax, True, artist_collection.lanes),
    blit=False,
)
# plt.show()
a.save("test.mp4", writer=animation.FFMpegWriter(fps=10), dpi=300)
