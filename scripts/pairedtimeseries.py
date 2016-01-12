
import numpy as np
from collections import OrderedDict


class PairedTimeSeries(object):

    def __init__(self, K):
        self.K = K
        self.X = OrderedDict()
        self.Y = OrderedDict()

    def add_x(self, x, t):
        self.X[t] = x
        return self

    def add_y(self, y, t):
        self.Y[t] = y
        return self

    def get_time_series(self):
        xs = list()
        ys = list()
        ts = set(self.X.keys()) & set(self.Y.keys())
        for t in ts:
            xs.append(self.X[t])
            ys.append(self.Y[t])
        return np.array(xs), np.array(ys)
