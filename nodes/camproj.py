
import numpy as np
from math import cos, sin
from itertools import product


class CameraProjection(object):

    def __init__(self, fov_x, fov_y):
        self.th_v = 0.5 * fov_x
        self.th_h = 0.5 * fov_y
        self.xis = self.get_xis(self.th_v, self.th_h)
        self.ez = np.array([0, 0, 1])

    def get_xis(self, th_v, th_h):
        ret_xis = list()
        for i, j in product([1, -1], [1, -1]):
            x = i * cos(th_h) * sin(th_v) / cos(th_v)
            y = j * sin(th_h)
            z = cos(th_h)
            ret_xis.append(np.matrix([x, y, z]).T)
        return ret_xis

    def get_lambdas(self, rot_qm, rot_cq, trans_qm, trans_cq):
        lmds = list()
        for xi in self.xis:
            top = np.inner((-(rot_qm * trans_cq + trans_qm)).A1, self.ez)
            bot = np.inner((rot_qm * rot_cq * xi).A1, self.ez)
            lmds.append(top / bot)
        return lmds

    def get_projection(self, rot_qm, rot_cq, trans_qm, trans_cq):
        rot_qm = np.matrix(rot_qm)
        rot_cq = np.matrix(rot_cq)
        trans_qm = np.matrix(trans_qm)
        trans_cq = np.matrix(trans_cq)
        lmds = self.get_lambdas(rot_qm, rot_cq, trans_qm, trans_cq)
        hom_qm = self.homogeneous(rot_qm, trans_qm)
        hom_cq = self.homogeneous(rot_cq, trans_cq)
        verts = list()
        for i in xrange(4):
            lxi = np.concatenate(
                (lmds[i] * self.xis[i], np.matrix([1])), axis=0)
            v = np.array(hom_qm * hom_cq * lxi)[:2]
            verts.append(v)
        verts = [verts[0], verts[1], verts[3], verts[2]]
        return verts

    def homogeneous(self, rot, trans):
        rt = np.concatenate((rot, trans), axis=1)
        hom = np.concatenate((rt, np.matrix([0, 0, 0, 1])), axis=0)
        return hom
