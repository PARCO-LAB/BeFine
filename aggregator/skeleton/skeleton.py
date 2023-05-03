"""
Skeleton format:

JSON
{
    "continuousState": [
        [ # Skeleton_0
            [x00, y00, z00], [x01, y01, z01], ..., [x0i, y0i, z0i], ...
        ],
        [ # Skeleton_1
            [x10, y10, z10], [x11, y11, z11], ..., [x1i, y1i, z1i], ...
        ],
        ...,
        [ # Skeleton_i
            [xi0, yi0, zi0], [xi1, yi1, zi1], ..., [xii, yii, zii], ...
        ],
        ...
    ],
    "timestamp": # message timestamp in ms,
    "timestamp_pub": # publish timestamp in ns,
}
"""


import numpy as np
from pandas import array
from scipy.spatial import distance


class Coords:
    @staticmethod
    def cdist(coords):
        d = distance.cdist(coords, coords, "euclidean")
        d[np.tril_indices(d.shape[0])] = np.inf
        return d

    @staticmethod
    def remove_outliers(coords, outlier_dist):
        d = Coords.cdist(coords)
        """
        # Remove the outliers
        index = np.argwhere(d > outlier_dist)
        coords_outlier = np.argwhere(np.bincount(index.flatten()) > 1)
        coords_to_remove = np.full(coords.shape[0], False)
        coords_to_remove[coords_outlier] = True
        return coords[~coords_to_remove]
        """
        # Take the coords that are not outliers.
        index = np.argwhere(d <= outlier_dist)
        """
        # Manage the min of distances.
        if index.shape[0] == 0:
            min_d = np.min(d)
            if min_d < (1.2 * outlier_dist):
                return coords[np.argmin(d, axis=0)]
        """
        coords_to_add = np.unique(index.flatten())
        return coords[coords_to_add]
            

class Skeleton:
    @staticmethod
    def to_np(skeletons):
        arr = [[e if len(e) > 0 else [np.nan] * 3 for e in obj] for obj in skeletons]
        return np.array(arr)

    @staticmethod
    def remove_nan(skeleton):
        nan = np.isnan(skeleton).any(axis=-1)
        nan = np.repeat(nan, skeleton.shape[-1], axis=0)
        nan = np.reshape(nan, skeleton.shape)
        return np.reshape(skeleton[~nan], (-1, skeleton.shape[-1]))

    @staticmethod
    def remove_empty(skeletons):
        if len(skeletons.shape) > 2:
            return np.array([skeleton for skeleton in skeletons if Skeleton.count_valid(skeleton) > 0])
        else:
            return skeletons if Skeleton.count_valid(skeletons) > 0 else np.array([])
        
    @staticmethod
    def count_valid(skeletons):
        nan = np.isnan(skeletons).any(axis=-1)
        return (~nan).sum(axis=-1)

    @staticmethod
    def center(skeletons, func=np.mean):
        if len(skeletons.shape) > 2:
            means = [func(Skeleton.remove_nan(skeleton_np), axis=0) for skeleton_np in skeletons]
            return np.array(means)
        else:
            mean = func(Skeleton.remove_nan(skeletons), axis=0)
            return mean

    @staticmethod
    def dist(skeleton1, skeleton2):
        return np.linalg.norm(skeleton1 - skeleton2, axis=-1)

    @staticmethod
    def dist_mean(skeleton1, skeleton2):
        d = Skeleton.dist(skeleton1, skeleton2)
        d = d[~np.isnan(d)]
        return np.mean(d)

    @staticmethod
    def dist_center(skeletons, func=np.mean):
        o = Skeleton.center(skeletons, func)
        o = np.repeat([o], skeletons.shape[-2], axis=-2)
        o = np.reshape(o, skeletons.shape)
        d = Skeleton.dist(skeletons, o)
        return d

    @staticmethod
    def dist_center_mean(skeletons, func=np.mean):
        d = Skeleton.dist_center(skeletons, func)
        d = d[~np.isnan(d)]
        return np.mean(d)

    @staticmethod
    def fusion(skeletons, topics, cameras=None, func=np.median, remove_outliers=True, outlier_dist=0.50):
        f = []

        # Remove outliers respect to the centers of the whole set of points of the skeletons to fuse.
        o = Skeleton.center(skeletons, func)
        o_mean = np.nanmean(o, axis=0)
        o_arr = np.repeat([o_mean], skeletons.shape[0], axis=0)
        o_arr = np.repeat([o_arr], skeletons.shape[-2], axis=-2)
        o_arr = np.reshape(o_arr, skeletons.shape)
        d = Skeleton.dist(skeletons, o_arr)
        mask = np.argwhere(d > 1.5)
        skeletons[mask[:, 0], mask[:, 1]] = np.array([np.nan] * skeletons.shape[-1])

        for kp_idx in range(skeletons.shape[1]):
            kps = skeletons[:, kp_idx, :]
            if kps.shape[0] == 0:
                f.append([np.nan] * kps.shape[1])
                continue
            kps = Skeleton.remove_nan(kps)
            if kps.shape[0] == 0:
                f.append([np.nan] * kps.shape[1])
                continue
            if cameras is not None: # Test
                d = distance.cdist(kps, cameras, "euclidean")
                d_argmin = np.unravel_index(np.argmin(d), d.shape)
                f.append(kps[d_argmin[0]])
            else:
                if remove_outliers and kps.shape[0] not in [0, 1]:
                    kps = Coords.remove_outliers(kps, outlier_dist)
                f.append(func(kps, axis=0))
        return np.array(f)


def test():
    import time
    KEY_SKELETON = "continuousState"
    KEY_MESSAGE = "timestamp"
    KEY_PUBLISH = "timestamp_pub"
    data = {
        KEY_SKELETON: [
            # Skeleton 1
            [
                [0, 0, 0], [], [], [3, 3, 3]
            ],
            # Skeleton 2
            [
                [1, 1, 1], [1, 1, 1], [], []
            ],
            # Skeleton 3
            [
                [1, 1, 1], [2, 2, 2], [], []
            ]
        ],
        KEY_MESSAGE: int(time.time() * 1e9),
        KEY_PUBLISH: int(time.time() * 1e9) * 1e-6
    }

    skeletons_np = Skeleton.to_np(data[KEY_SKELETON])
    print("Skeleton kps_to_np: \n", skeletons_np)
    print("Skeleton kps_to_np without nan: ")
    for skeleton_np in skeletons_np:
        print(Skeleton.remove_nan(skeleton_np))
    print("Skeleton center: \n", Skeleton.center(skeletons_np))
    print("Distance between skeleton 1 and 2: ", Skeleton.dist(skeletons_np[0], skeletons_np[1]))
    print("Mean distance between skeleton 0 and 1: ", Skeleton.dist_mean(skeletons_np[0], skeletons_np[1]))
    print("Mean distance between skeleton 0 and 1 check: ",
          Skeleton.dist_mean(Skeleton.center(skeletons_np)[0], Skeleton.center(skeletons_np)[1]))
    print("Mean distance between skeleton 1 and 2: ", Skeleton.dist_mean(skeletons_np[1], skeletons_np[2]))
    print("Mean distance between skeleton 1 and 2 check: ",
          Skeleton.dist_mean(Skeleton.center(skeletons_np)[1], Skeleton.center(skeletons_np)[2]))
    print("Skeletons fusion: ", Skeleton.fusion(skeletons_np))
    print("Distance center: ", Skeleton.dist_center(skeletons_np))
    print("Mean distance center: ", Skeleton.dist_center_mean(skeletons_np[0]))
    print("Count valid kps: ", Skeleton.count_valid(skeletons_np[0]))
    print("Count valid kps: ", Skeleton.count_valid(skeletons_np))

    s = skeletons_np.copy()
    d = Skeleton.dist_center(s, np.median)
    mask = np.argwhere(d > 1.5)
    s[mask[:, 0], mask[:, 1]] = np.array([np.nan] * s.shape[-1])
    print("Remove outliars: \n", s)

    s = skeletons_np.copy()
    o = Skeleton.center(s, np.median)
    o_mean = np.nanmean(o, axis=0)
    o_arr = np.repeat([o_mean], s.shape[0], axis=0)
    o_arr = np.repeat([o_arr], s.shape[-2], axis=-2)
    o_arr = np.reshape(o_arr, s.shape)
    d = Skeleton.dist(s, o_arr)
    mask = np.argwhere(d > 1.5)
    s[mask[:, 0], mask[:, 1]] = np.array([np.nan] * s.shape[-1])
    print("Remove outliars 2: \n", s)


if __name__ == '__main__':
    test()