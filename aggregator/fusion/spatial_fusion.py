"""

"""

import math
import numpy as np
from sklearn.cluster import DBSCAN, OPTICS, MeanShift, AffinityPropagation, AgglomerativeClustering
import scipy.cluster.hierarchy as hcluster
from sklearn.preprocessing import StandardScaler

from skeleton import Skeleton

KEYPOINT_COUPLES = [(4, 10), (3, 9)]
# KEYPOINT_COUPLES = [(0, 2), (3, 4)]


class SpatialFusion:
    @staticmethod
    def feature_extraction(skeletons):
        # Calculate centers
        centers = Skeleton.center(skeletons)
        centers_dist_origin = np.linalg.norm(centers, axis=1)

        # Calculate shapes
        shapes = []
        for couple in KEYPOINT_COUPLES:
            shapes.append(Skeleton.dist(skeletons[:, couple[0]], skeletons[:, couple[1]]))
        with np.testing.suppress_warnings() as sup:
            sup.filter(RuntimeWarning)
            shapes_mean = np.nan_to_num(np.nanmean(np.array(shapes), axis=0))

        num_kps = skeletons.shape[-2]
        skeletons = np.insert(skeletons, skeletons.shape[-1],
                              np.array([centers_dist_origin] * num_kps).transpose(), axis=-1)
        skeletons = np.insert(skeletons, skeletons.shape[-1],
                              np.array([shapes_mean] * num_kps).transpose(), axis=-1)
        return skeletons

    @staticmethod
    def remove_infinity(skeletons):
        skeletons_clean = []
        for i, skeleton in enumerate(skeletons):
            skeleton_clean = []
            for joint in skeleton:
                if np.any(joint > 300.0):
                    skeleton_clean.append([np.nan]*3)
                else:
                    skeleton_clean.append(joint)
            skeletons_clean.append(skeleton_clean)

        return np.array(skeletons_clean)

    @staticmethod
    def run(skeletons, cameras=None, mode="dbscan"):
        skeletons = SpatialFusion.remove_infinity(skeletons)
        valid_kps_count = Skeleton.count_valid(skeletons)

        # Filter skeletons for kps amount. 
        # skeletons_valid = [(s, valid_kps_count[i]) for i, s in enumerate(skeletons) if valid_kps_count[i] > 9]
        # if len(skeletons_valid) == 0:
        #     return [], np.array([])
        # skeletons, valid_kps_count = zip(*skeletons_valid)
        # skeletons = np.array(skeletons)

        # Remove for shape
        skeletons_within_shape = []
        valid = []
        for i, skeleton in enumerate(skeletons):
            s = Skeleton.remove_nan(skeleton)
            if skeleton.shape[0] > 0:
                min_skeleton = np.min(s, axis=0)
                max_skeleton = np.max(s, axis=0)

                if max_skeleton[0] - min_skeleton[0] < 2.0 and max_skeleton[1] - min_skeleton[1] < 2.0 and max_skeleton[2] - min_skeleton[2] < 3.0 and max_skeleton[2] - min_skeleton[2] > 0.7 and max_skeleton[2] < 2.70:
                    skeletons_within_shape.append(skeleton)
                    valid.append(valid_kps_count[i])

        if len(skeletons_within_shape) == 0:
            return [], np.array([])
        skeletons = np.array(skeletons_within_shape)
        valid_kps_count = valid 

        skeletons_shape = list(skeletons.shape)
        skeletons_shape[-1] += 1  # with labels
        label_idx = skeletons_shape[-1] - 1

        # Remove outliers. 
        d = Skeleton.dist_center(skeletons, np.median)
        mask = np.argwhere(d > 1.5)
        skeletons[mask[:, 0], mask[:, 1]] = np.array([np.nan] * skeletons.shape[-1])

        # Preprocessing
        data_preprocessed = SpatialFusion.feature_extraction(skeletons)
        # data_preprocessed = skeletons
        data_reshape = data_preprocessed.reshape((-1, data_preprocessed.shape[-1]))
        data = Skeleton.remove_nan(data_reshape)
        if (data.shape[0] == 0):
            return [], np.array([])

        # Clustering
        average_kps_count = np.mean(valid_kps_count)
        min_samples = max(int(average_kps_count / 2), 1)
        min_eps = 0.4
        if mode == "dbscan":      
            average_dist_center = Skeleton.dist_center_mean(skeletons)
            cl = DBSCAN(eps=max(average_dist_center, min_eps), min_samples=min_samples).fit(data)
        elif mode == 'optics':
            cl = OPTICS(eps=min_eps, min_samples=min_samples).fit(data)
        elif mode == 'meanshift':
            cl = MeanShift(bandwidth=2).fit(data)
        elif mode == 'affinity':          
            cl = AffinityPropagation().fit(data)     
        elif mode == 'agglomerative':
            cl = AgglomerativeClustering(distance_threshold=min_eps, n_clusters=None).fit(data)
        else:
            raise ValueError("mode string not recognized")
        
        labels = cl.labels_

        skeletons = skeletons.reshape((-1, skeletons.shape[-1]))
        nan = np.isnan(skeletons).any(axis=-1)
        skeletons = np.insert(skeletons, skeletons.shape[-1],
                              [np.nan] * data_reshape.shape[0], axis=-1)
        skeletons[~nan, label_idx] = labels

        # Remove outliers obtained from clustering
        # outliers = skeletons[:, label_idx] == -1
        # skeletons[outliers] = np.array([np.nan] * skeletons_shape[-1])

        skeletons = skeletons.reshape(skeletons_shape)
        skeletons = np.delete(skeletons, label_idx, axis=-1)

        accumulate_count = 0
        clusters = []
        for skeleton_valid_kps_count in valid_kps_count:
            skeleton_labels = labels[accumulate_count:accumulate_count + skeleton_valid_kps_count]
            skeleton_labels = [e for e in skeleton_labels if e != -1]
            if len(skeleton_labels) != 0:
                clusters.append(math.ceil(np.mean(skeleton_labels)))
            else:
                clusters.append(-1)
            accumulate_count += skeleton_valid_kps_count
        if len(clusters) == 1:
            clusters = [0]

        merged_skeletons = []
        # for cluster in set([e for e in clusters if e != -1]): # Remove skeleton outliar from clustering
        for cluster in set([e for e in clusters]):
            indexes_of_cluster = np.argwhere(np.array(clusters) == cluster).flatten()
            if indexes_of_cluster.shape[0] == 1:
                merged_skeletons.append(skeletons[indexes_of_cluster[0]])
            else:
                merged_skeletons.append(Skeleton.fusion(skeletons[indexes_of_cluster], cameras))
        
        return clusters, np.array(merged_skeletons)


def test():
    data = [
        # Skeleton 1
        [
            [4, 4, 4], [], [], [4, 4, 4], [5, 5, 5], [5, 5, 5], []
        ],
        # Skeleton 2
        [
            [1, 1, 1], [], [1, 1, 1], [], [5, 5, 5], [5, 5, 5], []
        ],
        # Skeleton 3
        [
            [2, 2, 2], [], [2, 2, 2], [], [5, 5, 5], [5, 5, 5], []
        ],
        # Skeleton 3
        [
            [2, 2, 2], [], [2, 2, 2], [], [5, 5, 5], [5, 5, 5], []
        ],
        # Skeleton 4
        [
            [5, 5, 5], [5, 5, 5], [], [], [5, 5, 5], [5, 5, 5], [10, 10, 10]
        ]
    ]
    skeletons_np = Skeleton.to_np(data)
    clusters, fusion = SpatialFusion.run(skeletons_np)
    print("Assigned clusters: ", clusters)
    print("Fusion result: \n", fusion)


if __name__ == '__main__':
    test()
