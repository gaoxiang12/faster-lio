# coding="utf8"
import numpy as np
from matplotlib import pyplot as plt

data = np.loadtxt('./kdtree.ivox.csv', delimiter=',')
print(data)

plt.subplot(1, 2, 1)
plt.plot(data[:, 0], data[:, [1, 3, 5, 7, 9, 11, 13, 15]], marker='o')

plt.xscale('log')
plt.yscale('log')
plt.legend(['iVox-Linear', 'iVox-PHC', 'ik-d tree', 'flann', 'R-tree', 'faiss-IVF', 'nmslib hnws', 'nanoflann hnws'])
plt.xlabel('Num of Points')
plt.ylabel('Time Usage (ms)')
plt.title('Insert Time w.r.t. Num of Points')
plt.grid()

plt.subplot(1, 2, 2)
plt.plot(data[:, 0], data[:, [2, 4, 6, 8, 10, 12, 14, 16]], marker='o', linestyle='--')
plt.xscale('log')
plt.yscale('log')
plt.xlabel('Num of Points')
plt.ylabel('Time Usage (ms)')
plt.legend(['iVox-Linear', 'iVox-PHC', 'ik-d tree', 'flann', 'R-tree', 'faiss-IVF', 'nmslib hnws', 'nanoflann hnws'])
plt.title('Knn Time w.r.t. Num of Points')
plt.grid()

plt.show()
