import sys
sys.path.append("../..")
from py3d import *
import numpy as np
import time

class Timer(object):
    def __init__(self, name=None):
        self.name = name

    def __enter__(self):
        self.tstart = time.time()

    def __exit__(self, type, value, traceback):
        if self.name:
            print('[%s]' % self.name)
        print('Elapsed: %.3f' % (time.time() - self.tstart))

with Timer("Test Vector3dVector"):
    x = np.random.random((2000000,3))
    y = open3d.Vector3dVector(x)
    # print(y)
