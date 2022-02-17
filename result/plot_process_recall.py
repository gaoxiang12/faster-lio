# coding="utf8"
import numpy as np
from matplotlib import pyplot as plt
from operator import itemgetter

# tested in amd r7-5800x
process_ivox = [0.04886, 0.00019, 0.897, 2.235, 0.000195, 0.0112, 0.342, 0.00038, 0.000175, 0.008235, 0.00029]
recall_ivox = [0.905, 0.591, 0.926, 0.99, 0.450000, 0.8100, 0.920, 0.569688, 0.254986, 0.832909, 0.705829]
process_ikdtree = [0.536, 0.146, 0.848, 0.951, 0.754]
recall_ikdtree = [0.669, 0.27, 0.977, 1, 0.888878]
process_pcl = [0.363, 0.216, 0.114, 0.032355, 0.00089]
recall_pcl = [0.99, 0.979, 0.878, 0.752, 0.595664]
process_faiss = [1.05037, 1.00586, 2.12313, 1.73046, 1.00289]
recall_faiss = [0.946958, 0.856685, 0.999936, 0.993149, 0.648557]
process_nmslib_hnws = [1.10155, 1.10576]
recall_nmslib_hnws = [0.999031, 0.939438]
process_nano_hnws = [1.40931, 1.23796, 1.05759, 0.51051, 0.8211, 0.280825, 0.110685, 0.03312, 0.006765]
recall_nano_hnws = [0.990816, 0.964372, 0.914932, 0.719922, 0.807666, 0.619177, 0.554345, 0.481125, 0.434597]


def get_sorted_pr(process, recall):
    '''计算按recall排序后的precess recall'''
    data = [(process[i], recall[i] * 100) for i in range(len(process))]
    data = sorted(data, key=itemgetter(1))
    return np.asarray([list(data[i]) for i in range(len(process))], dtype=np.float)


sorted_pr_ivox = get_sorted_pr(process_ivox, recall_ivox)
plt.plot(sorted_pr_ivox[:, 1], sorted_pr_ivox[:, 0], marker='o')

sorted_pr_ikdtree = get_sorted_pr(process_ikdtree, recall_ikdtree)
plt.plot(sorted_pr_ikdtree[:, 1], sorted_pr_ikdtree[:, 0], marker='o')

sorted_pr_pcl = get_sorted_pr(process_pcl, recall_pcl)
plt.plot(sorted_pr_pcl[:, 1], sorted_pr_pcl[:, 0], marker='o')

sorted_pr_faiss = get_sorted_pr(process_faiss, recall_faiss)
plt.plot(sorted_pr_faiss[:, 1], sorted_pr_faiss[:, 0], marker='o')

sorted_pr_nmslib_hnws = get_sorted_pr(process_nmslib_hnws, recall_nmslib_hnws)
plt.plot(sorted_pr_nmslib_hnws[:, 1], sorted_pr_nmslib_hnws[:, 0], marker='o')

sorted_pr_nano_hnws = get_sorted_pr(process_nano_hnws, recall_nano_hnws)
plt.plot(sorted_pr_nano_hnws[:, 1], sorted_pr_nano_hnws[:, 0], marker='o')

plt.yscale('log')
plt.legend(['iVox', 'ik-d tree', 'flann', 'faiss-IVF', 'nmslib hnws', 'nanoflann hnws'])
plt.xlabel('Recall rate (%)')
plt.ylabel('Processing time (ms)')
plt.title('Processing time w.r.t. recall rate')
plt.grid()
plt.show()
