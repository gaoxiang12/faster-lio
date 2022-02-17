# coding="utf8"
import numpy as np
from matplotlib import pyplot as plt

labels = ['avia1', 'avia2', 'avia3', 'avia4', 'avia5', 'avia6', 'nclt1', 'nclt2', 'utbm1', 'utbm2']

# fps
fps_ivox_def_amd = [1820.90, 1720.79, 409.204, 387.982, 641.977, 1785.08, 231.763, 302.876, 223.648, 198.338]
fps_ivox_phc_amd = [1751.92, 1868.11, 357.531, 341.898, 627.306, 1728.50, 218.018, 287.928, 228.553, 197.226]
fps_fastlio2_amd = [1612.35, 1642.74, 222.829, 206.172, 502.121, 1164.00, 151.540, 126.564, 105.487, 116.181]

fps_ivox_def_intel = [656.781, 693.343, 172.365, 144.881, 215.141, 679.492, 116.313, 133.697, 101.061, 93.4607]
fps_ivox_phc_intel = [621.447, 699.480, 157.677, 136.818, 218.938, 619.714, 117.117, 135.905, 96.6023, 91.2655]
fps_fastlio2_intel = [306.091, 348.614, 48.4737, 46.0621, 97.2555, 295.081, 36.7999, 31.8538, 28.6985, 30.5153]

# iekf 用时
iekf_ivox_def_amd = [0.36302, 0.339552, 1.24889, 1.35735, 0.574273, 0.363231, 2.91577, 2.10967, 1.99293, 2.24326]
iekf_ivox_phc_amd = [0.379194, 0.344433, 1.52312, 1.62747, 0.61055, 0.376235, 2.99102, 2.14127, 2.56342, 2.20624]
iekf_fastlio2_amd = [0.438023, 0.408461, 3.01613, 3.35717, 0.979082, 0.62697, 4.69048, 5.96732, 6.62869, 5.89594]

iekf_ivox_def_intel = [
    0.978076,
    0.899829,
    2.61272,
    3.66871,
    2.02132,
    0.932981,
    5.45205,
    4.71979,
    4.858,
    5.09936,
]

iekf_ivox_phc_intel = [
    1.03404,
    0.874754,
    3.09548,
    3.9395,
    1.951,
    1.01063,
    5.30898,
    4.57925,
    5.20434,
    5.2672,
]

iekf_fastlio2_intel = [
    2.36772,
    1.97881,
    14.0422,
    14.8879,
    5.26109,
    2.41927,
    20.2533,
    23.0207,
    24.5173,
    22.6144,
]

# 第一个图
fig, (ax1, ax2) = plt.subplots(1, 2)
x = np.arange(len(fps_ivox_phc_amd))
total_width = 0.8
n = 3

width = total_width / n
x = x - (total_width - width) / 2

ax1.set_xlabel('dataset')
ax1.set_ylabel('Frame per second')

# amd
ax1.bar(x - width, fps_ivox_def_amd, width=width, label='Faster-LIO AMD', color='lightcoral')
ax1.bar(x, fps_ivox_phc_amd, width=width, label='Faster-LIO PHC AMD', color='lightblue', tick_label=labels)
ax1.bar(x + width, fps_fastlio2_amd, width=width, label='FastLIO2 AMD', color='lawngreen')

# intel
ax1.bar(x - 0.5 * width, fps_ivox_def_intel, width=width, label='Faster-LIO Intel', color='firebrick')
ax1.bar(x + 0.5 * width, fps_ivox_phc_intel, width=width, label='Faster-LIO PHC Intel', color='cornflowerblue')
ax1.bar(x + 1.5 * width, fps_fastlio2_intel, width=width, label='FastLIO2 Intel', color='seagreen')

# for a, b in zip(x, fps_ivox_def_amd):
#     ax1.text(a - width, b, b, ha='center', va='bottom', fontsize=8.0)
# for a, b in zip(x, fps_ivox_phc_amd):
#     ax1.text(a, b, b, ha='center', va='bottom', fontsize=8.0)
# for a, b in zip(x, fps_fastlio2_amd):
#     ax1.text(a + width, b, b, ha='center', va='bottom', fontsize=8.0)
#
# for a, b in zip(x, fps_ivox_def_intel):
#     ax1.text(a - 0.5 * width, b, b, ha='center', va='bottom', fontsize=8.0)
# for a, b in zip(x, fps_ivox_phc_intel):
#     ax1.text(a + 0.5 * width, b, b, ha='center', va='bottom', fontsize=8.0)
# for a, b in zip(x, fps_fastlio2_intel):
#     ax1.text(a + 1.5 * width, b, b, ha='center', va='bottom', fontsize=8.0)

ax1.legend()
ax1.set_title("FPS of LIO in each sequence")

# 第二个图
ax2.set_yscale('log')
x = np.arange(len(iekf_ivox_phc_amd))
fontsize = 12.0

ax2.plot(x, iekf_ivox_def_amd, marker='o', color='lightcoral')
ax2.plot(x, iekf_ivox_phc_amd, marker='o', color='lightblue')
ax2.plot(x, iekf_fastlio2_amd, marker='o', color='lawngreen')
ax2.plot(x, iekf_ivox_def_intel, marker='o', color='firebrick')
ax2.plot(x, iekf_ivox_phc_intel, marker='o', color='cornflowerblue')
ax2.plot(x, iekf_fastlio2_intel, marker='o', color='seagreen')

# 字符
# for a, b in zip(x, iekf_fastlio2_amd):
#     plt.text(a, b, b, ha='center', va='bottom', fontsize=fontsize)
# for a, b in zip(x, iekf_ivox_def_amd):
#     plt.text(a, b, b, ha='center', va='bottom', fontsize=fontsize)
# for a, b in zip(x, iekf_ivox_phc_amd):
#     plt.text(a, b, b, ha='center', va='bottom', fontsize=fontsize)
#
# for a, b in zip(x, iekf_fastlio2_intel):
#     plt.text(a, b, b, ha='center', va='bottom', fontsize=fontsize)
# for a, b in zip(x, iekf_ivox_def_intel):
#     plt.text(a, b, b, ha='center', va='bottom', fontsize=fontsize)
# for a, b in zip(x, iekf_ivox_phc_intel):
#     plt.text(a, b, b, ha='center', va='bottom', fontsize=fontsize)

ax2.set_xlabel('dataset')
ax2.set_ylabel('Time Usage (ms)')
ax2.legend(['Faster-LIO AMD', 'Faster-LIO PHC AMD', 'FastLIO2 AMD', 'Faster-LIO Intel', 'Faster-LIO PHC Intel',
            'FastLIO2 Intel'])
ax2.grid(True)
ax2.set_xticks(x)
ax2.set_xticklabels(labels)
ax2.set_title("IEKF + ICP Ave. Time Usage")

plt.show()
