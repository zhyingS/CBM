import copy
import numpy as np
from scipy.spatial.transform import Rotation
import argparse

class CBM():
    # All the transform are defined in right hand coordinates

    def __init__(self, args=None):
        if args==None:
            self.args = self.parser()
        else:
            self.args = args

    def parser(self):
        parser = argparse.ArgumentParser()
        parser.add_argument('--sigma1',default=10*np.pi/180,
                            help='rad')
        parser.add_argument('--sigma2',default=3,
                            help='m')
        parser.add_argument('--sigma3',default=1,
                        help='m')
        parser.add_argument('--absolute_dis_lim',default=20,
            help='m')

        args=parser.parse_args()

        return args

    def __call__(self, Ego, Cav, transform):
        # Unify the orientation
        Ego, Cav = self.Uni_Ori(Ego, Cav, transform)

        # Construct local context
        P, Q = self.CLC(Ego), self.CLC(Cav)
        self.m, self.n = len(P), len(Q)

        # Local matching
        M, M_G = self.LM(P, Q)

        # Global matching
        A_ = self.GM(M, M_G, Ego, Cav)

        # Convert matching matrix to the form [[i,j]]
        m = np.where(A_ > 0)
        matching = np.hstack((m[0].reshape(-1, 1), m[1].reshape(-1, 1)))

        return matching

    def Uni_Ori(self, Ego, Cav, transform):
        # Ego, Cav: Mx7, Nx7
        R, t = transform[:-1, :-1], transform[:-1, -1].reshape(-1, 1)
        angle = Rotation.from_matrix(R).as_euler('xyz', degrees=False)
        Cav[:, -1] += angle[-1]
        Cav[:, 0:3] = (np.dot(R, Cav[:, 0:3].T) + t).T
        Ego[:, -1] = np.mod(Ego[:, -1], np.pi)
        Cav[:, -1] = np.mod(Cav[:, -1], np.pi)
        v = Ego[:, -1]
        index_ego = np.where((v > np.pi / 2) & (v < np.pi * 3 / 2))[0]
        if index_ego.shape[0] != 0:
            Ego[index_ego, -1] -= np.pi
        v = Cav[:, -1]
        index_cav = np.where((v > np.pi / 2) & (v < np.pi * 3 / 2))[0]
        if index_cav.shape[0] != 0:
            Cav[index_cav, -1] -= np.pi
        return Ego, Cav

    def GM(self, M, M_G, Ego, Cav):
        m, n = self.m, self.n
        count = np.zeros((m, n))
        A = {}
        for i in range(m):
            for j in range(n):
                Gij = np.zeros((m, n))
                Mij = M['{}_{}'.format(i, j)]

                column_sum = np.sum(Mij, axis=1).reshape(-1, 1)
                row_sum = np.sum(Mij, axis=0).reshape(1, -1)
                sum_ = column_sum + row_sum
                sum_mask = (sum_ == 2)
                sum_mask = (sum_mask * Mij==1)
                G_set = np.where(sum_mask != 0)

                for k_ in range(G_set[0].shape[0]):
                    Gij += M_G['{}_{}'.format(G_set[0][k_], G_set[1][k_])]
                Gij_ = (Gij*(Mij-sum_mask) >= 1)
                Mij_=((sum_mask+Gij_)>=1)

                Aij = copy.deepcopy(Mij_)
                for k in range(m):
                    for h in range(n):
                        if np.sum(Mij_[k, :]) > 1 or np.sum(Mij_[:, h]) > 1:
                            Aij[k, h] = 0
                D_=np.where(Aij!=0)
                D=np.linalg.norm(Ego[D_[0],:]-Cav[D_[1],:],axis=1)

                if np.mean(D)<=self.args.absolute_dis_lim:
                    count[i, j] = np.sum(Aij)
                else:
                    count[i, j] = 0

                A.update({'{}_{}'.format(i, j): Aij})

        # if i_j_ is not unique, who last occured will be selected.
        ij=np.where(np.max(count)==count)
        A_ = A['{}_{}'.format(ij[0][-1], ij[1][-1])]

        return A_

    def LM(self, P, Q):
        m, n = self.m, self.n
        sigma1, sigma2, sigma3 = self.args.sigma1, self.args.sigma2, self.args.sigma3
        M, M_G={}, {}
        for i in range(m):
            for j in range(n):
                Mij, Mij_G = np.zeros((m, n)), np.zeros((m, n))
                p, q = P[i], Q[j]
                a = np.dot(p.T, q)
                b = np.dot(np.linalg.norm(p, axis=0).reshape(-1, 1), np.linalg.norm(q, axis=0).reshape(1, -1))
                c = abs(a / b)
                c[c > 1] = 1
                Sz1 = np.arccos(c) / sigma1

                sta = np.where(Sz1 <= 1)
                Aij = np.vstack((sta[0], sta[1]))
                Aij = np.hstack((Aij, np.array([[i], [j]])))
                p1, q1 = p[:, Aij[0, :]], q[:, Aij[1, :]]
                Sz2 = np.linalg.norm(p1 - q1, axis=0, ord=1)
                for k in range(Aij.shape[1]):
                    if Sz2[k] <= sigma2:
                        Mij[Aij[0, k], Aij[1, k]] = 1
                    if Sz2[k] <= sigma3:
                        Mij_G[Aij[0, k], Aij[1, k]] = 1
                if np.sum(Mij) <= 1:
                    Mij = np.zeros((m, n))              

                M.update({'{}_{}'.format(i, j): Mij})
                M_G.update({'{}_{}'.format(i, j): Mij_G})

        return M, M_G

    def CLC(self, Ego):
        # construct local context
        # Input: Mx7 or Nx7 numpy array
        X = Ego[:, [0, 1, -1]]
        P = {}
        for i in range(X.shape[0]):
            X_ = X[:, 0:-1] - X[i, 0:-1]
            theta = X[i, -1]
            c, s = np.cos(theta), np.sin(theta)
            R = np.array([[c, -s], [s, c]])
            Pi = np.dot(X_, R)
            Pi = Pi.T
            P.update({i: Pi})
        return P

    pass
