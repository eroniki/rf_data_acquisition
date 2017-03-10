import numpy as np
from scipy import io as scio


data1 = np.load("test_1489093174971513032.npy")
data2 = np.load("test_1489096095542711019.npy")

data_total = np.vstack([data1, data2])

print data_total.shape, np.mean(data_total)

scio.savemat("hancock_03_09.mat", {"data_total":data_total})
