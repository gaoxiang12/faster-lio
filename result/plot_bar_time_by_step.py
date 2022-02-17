#coding=utf8
import numpy as np
from matplotlib import pyplot as plt

columns = [u"IEKF+ICP", u"Undistortion", u"Downsample", u"Preprocess", u"Other"]
colors = ["firebrick", "dodgerblue", "darkorchid", "darkseagreen", "deepskyblue"]
rows = ["nclt_1", "nclt_2", "nclt_3", "nclt_4", "nclt_5", "utbm_2", "utbm_3", "utbm_4", "utbm_5", "utbm_6", "utbm_7", "utbm_8", "utbm_9"]

iVoxDefault = np.array([
[7.70328, 1.73245, 0.682075, 0.689807, 0.605588],
[6.57643, 1.53664, 0.599698, 0.600485, 0.474317],
[8.18002, 1.13471, 0.444477, 0.636257, 0.491736],
[8.50082, 1.8575, 0.736798, 0.714855, 0.634127],
[8.67173, 1.11188, 0.428228, 0.627745, 0.473817],
[5.469248, 1.285808, 0.461443, 2.080929, 0.521219],
[5.451835, 1.33575, 0.482405, 2.082823, 0.514342],
[5.539294, 1.31138, 0.472075, 2.118541, 0.560482],
[6.05517, 1.3925, 0.50487, 2.229838, 0.576102],
[5.730129, 1.376845, 0.498655, 2.198301, 0.56271],
[5.728699, 1.357128, 0.487709, 2.167314, 0.521744],
[5.412322, 1.291695, 0.461585, 2.006955, 0.512295],
[5.40148, 1.253372, 0.440815, 1.862076, 0.585229]
])

FastLio2 = np.array([
[17.1688, 1.79367, 0.75351, 0.773019, 0.820901],
[13.1976, 1.49159, 0.614052, 0.627339, 0.967219],
[13.8853, 1.44621, 0.607053, 0.616823, 0.921014],
[13.6501, 1.44434, 0.614458, 0.661158, 0.851744],
[12.2823, 1.38767, 0.575838, 0.631142, 0.76935],
[19.3468, 3.39617, 1.3282, 2.28693, 1.6094],
[19.7142, 3.67307, 1.39273, 2.26705, 1.80575],
[19.9745, 3.45101, 1.35079, 2.27994, 2.02376],
[20.6284, 3.66144, 1.431, 2.40762, 1.68174],
[20.3177, 3.72494, 1.43581, 2.45359, 1.89906],
[20.1057, 3.59717, 1.3947, 2.31395, 1.77378],
[20.2965, 3.4561, 1.3429, 2.20788, 1.79992],
[21.0653, 3.33586, 1.29085, 2.04243, 2.55246]
])

xticks = np.arange(len(iVoxDefault))
fig, ax = plt.subplots(figsize=(10, 7))
for i in range(len(columns)):
    #ax.bar(xticks, iVoxDefault[:, 0], label=columns[0])
    ax.bar(xticks, iVoxDefault[:, i], width=0.25, color=colors[i], bottom=iVoxDefault[:, :i].sum(axis=1), label=columns[i])
    ax.bar(xticks + 0.3, FastLio2[:, i], width=0.25, color=colors[i], bottom=FastLio2[:, :i].sum(axis=1))

ax.set_title("Time usage of each step", fontsize=15)
ax.set_xlabel("datasets")
ax.set_ylabel("Time/ms")
ax.legend()

ax.set_xticks(xticks + 0.15)
ax.set_xticklabels(rows)

plt.show()
