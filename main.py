import random
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import ConnectionPatch
from CBM import CBM as CBM_


def main(Ego_box, Cav_box):
    # Input: Ego box: Mx7 (x,y,z,h,w,l,theta) unit: m and rad
    #        Cav box: Nx7 (x,y,z,h,w,l,theta)

    CBM = CBM_()
    matching = CBM(Ego_box, Cav_box, transform=np.eye(4))

    return matching


def set_ax(ax):
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')

def plot_points(Ego, Cav, matching, GT_matching):
    colors = {'car': 'dodgerblue', 'bicycle': 'green', 'motorcycle': 'orange', 'bus': 'turquoise',
              'truck': 'yellow', 'tricycle': 'hotpink', 'pedestrian': 'red'}
    fig, (ax1, ax2) = plt.subplots(1, 2, sharex=True, sharey=True)
    ax1.set_aspect(1)
    ax2.set_aspect(1)

    for i in matching:
        con = ConnectionPatch(xyA=(Ego[i[0], 0], Ego[i[0], 1]), xyB=(Cav[i[1], 0], Cav[i[1], 1]),
                              coordsA="data", coordsB="data", axesA=ax1, axesB=ax2, color="green")
        ax2.add_artist(con)

    for i in GT_matching:
        a = np.all(matching == i.reshape(1, -1), axis=1)
        if np.sum(a) == 0:
            con = ConnectionPatch(xyA=(Ego[i[0], 0], Ego[i[0], 1]), xyB=(Cav[i[1], 0], Cav[i[1], 1]),
                                  coordsA="data", coordsB="data", axesA=ax1, axesB=ax2, color="red")
            ax2.add_artist(con)
    ax1.scatter(Ego[:, 0], Ego[:, 1])
    ax2.scatter(Cav[:, 0], Cav[:, 1])
    set_ax(ax1)
    set_ax(ax2)
    ax1.set_title('Objects detected by Ego')
    ax2.set_title('Objects detected by Helper')

    plt.show()


if __name__ == '__main__':
    # Mx7 (x,y,z,h,w,l,phi)
    # Generate two group of objects in the bounding box manner
    Ego_box = np.random.rand(50, 7) * 200 - 100
    Cav_box = np.random.rand(5, 7) * 200 - 100

    # generate ground truth matching correspondences
    co_visible_num = 2
    a = random.sample(range(Ego_box.shape[0]), co_visible_num)
    b = random.sample(range(Cav_box.shape[0]), co_visible_num)
    GT_matching = np.hstack((np.asarray(a).reshape(-1, 1), np.asarray(b).reshape(-1, 1)))

    Cav_box[GT_matching[:, 1], :] = Ego_box[GT_matching[:, 0], :]

    # Match the two groups of boxes by CBM
    matching = main(Ego_box, Cav_box)

    # Visulaization
    plot_points(Ego_box, Cav_box, matching, GT_matching)
