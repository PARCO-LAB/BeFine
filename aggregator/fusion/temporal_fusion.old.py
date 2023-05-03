"""

"""

import numpy as np
from scipy.spatial import distance_matrix
from scipy.optimize import linear_sum_assignment

from skeleton import Skeleton


class TemporalFusion:
    def __init__(self):
        self.history = None

    def run(self, skeletons, mode="linear_sum_assign"):
        skeletons = Skeleton.remove_empty(skeletons)
        if np.isnan(skeletons).all() or len(skeletons) == 0:
            return self
        if self.history is None:
            self.history = {"h" + str(i): e for i, e in enumerate(skeletons)}
            return self
        if mode == "linear_sum_assign":
            history = np.array(list(self.history.values()))
            idx_to_name = dict(zip(range(history.shape[0]), self.history.keys()))
            history_centers = Skeleton.center(history)
            
            skeleton_centers = Skeleton.center(skeletons)
            cost_matrix = distance_matrix(skeleton_centers, history_centers)
            row_ind, col_ind = linear_sum_assignment(cost_matrix)
           
            

            # Manage update
            for i in range(row_ind.shape[0]):
                self.history[idx_to_name[col_ind[i]]] = skeletons[row_ind[i]]

            # Manage delete
            keys_to_delete = set(range(len(self.history))).difference(set(col_ind))
            for i in keys_to_delete:
                self.history.pop(idx_to_name[i], None)

            # Manage addition
            idxs_to_add = set(range(skeletons.shape[0])).difference(set(row_ind))
            for i in idxs_to_add:
                new_key = "h" + str(int(list(self.history.keys())[-1][1:]) + 1)
                self.history[new_key] = skeletons[i]

            return self
        elif mode == "argmin":
            raise ValueError("argmin mode not implemented")
        else:
            raise ValueError("mode string not recognized")

    def get_history(self):
        return [{"keypoints": self.history[body_id].tolist(), "body_id": body_id} 
            for body_id in self.history] if self.history else []


def test():
    data = [
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
    ]
    skeletons_np = Skeleton.to_np(data)
    tf = TemporalFusion()
    print("Step 1: ", tf.run(skeletons_np).history)
    print("Step 2: ", tf.run(np.array([skeletons_np[0]])).history)
    print("Step 3: ", tf.run(np.array([skeletons_np[1]])).history)
    print("Step 4: ", tf.run(np.array([skeletons_np[2]])).history)
    data2 = [
        # Skeleton 1
        [
            [1, 1, 1], [1, 1, 1], [], []
        ],
        # Skeleton 2
        [
            [0, 0, 0], [], [], [3, 3, 3]
        ],
    ]
    skeletons_np = Skeleton.to_np(data2)
    print("Step 5: ", tf.run(skeletons_np).history)


if __name__ == '__main__':
    test()
