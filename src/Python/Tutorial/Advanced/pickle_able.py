# Open3D: www.open3d.org
# The MIT License (MIT)
# See license file or visit www.open3d.org for details

import sys
sys.path.append("../../")
from py3d import *
import numpy as np
import pickle

if __name__ == "__main__":

    vd = DoubleVector([1, 2, 3])
    print(vd)
    data = pickle.dumps(vd, pickle.HIGHEST_PROTOCOL)
    vd2 = pickle.loads(data)
    print(vd2)

    source = read_point_cloud("../../TestData/ICP/cloud_bin_0.pcd")
    print(np.asarray(source.points)[0:10])
    # f = open('store.pckl', 'wb')
    # pickle.dump(source, f)
    # f.close()
    # f = open('store.pckl', 'rb')
    # source2 = pickle.load(f)
    # f.close()

    data = pickle.dumps(source, pickle.HIGHEST_PROTOCOL)
    source2 = pickle.loads(data)
    print(np.asarray(source2.points)[0:10])
