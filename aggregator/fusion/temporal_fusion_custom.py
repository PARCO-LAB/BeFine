"""

"""

import numpy as np
from scipy.spatial import distance_matrix
from scipy.optimize import linear_sum_assignment

from skeleton import Skeleton

MAX_QUEUE_SIZE = 15
COUNT_INVALID = 4


class TemporalFusion:

    def __init__(self):
        self.history = None

    def get_used_ids(self):
        return [int(e[1:]) for e in self.history]

    def get_available_id(self):
        used_ids = self.get_used_ids()
        i = 0
        while True:
            if i not in used_ids:
                return i
            i += 1

    def queue_size(self):
        if self.history is None:
            return 0
        sizes = [len(self.history[e]) for e in self.history]
        if len(sizes) == 0:
            return 0
        return min(sizes)

    def get_skeleton_history_map(self, skeletons):
        map_list = []
        body_ids = list(self.history.keys())
        new_id = self.get_available_id()
        new_body_ids = []
        for t in range(self.queue_size()):
            history_i = {body_id: self.history[body_id][t] for body_id in self.history
                         if self.history[body_id][t] is not None}
            if len(history_i) == 0:
                continue
            history = np.array(list(history_i.values()))
            idx_to_name = dict(zip(range(history.shape[0]), history_i.keys()))
            history_centers = Skeleton.center(history)

            skeleton_centers = Skeleton.center(skeletons)
            cost_matrix = distance_matrix(skeleton_centers, history_centers)
            row_ind, col_ind = linear_sum_assignment(cost_matrix)

            # Manage update
            current_map = {}
            update_idx = []
            for i in range(row_ind.shape[0]):
                if cost_matrix[row_ind[i], col_ind[i]] <= 0.6:
                   current_map[row_ind[i]] = idx_to_name[col_ind[i]]
                   update_idx.append(row_ind[i])

            # Manage addition
            idxs_to_add = set(range(skeletons.shape[0])).difference(set(update_idx))
            for i, idx in enumerate(idxs_to_add):
                new_body_id = "h" + str(new_id + i)
                current_map[idx] = new_body_id
                body_ids.append(new_body_id)
                new_body_ids.append(new_body_id)
            map_list.append(current_map)

        body_ids = list(set(body_ids))
        new_body_ids = list(set(new_body_ids))

        body_ids_for_skeleton_idx = [[m[s_idx] for m in map_list if s_idx in m] for s_idx in range(len(skeletons))]
        new_body_ids_for_skeleton_idx = []
        for b_ids_s_i in body_ids_for_skeleton_idx:
            contains_new_body_id = False
            for b_i in b_ids_s_i:
                if b_i in new_body_ids:
                    contains_new_body_id = True
            
            if contains_new_body_id:
                b_ids = set(b_ids_s_i).difference(set(new_body_ids))
                if len(b_ids) > 0:
                    b_ids = list(b_ids)
                    b_id = max(b_ids, key=b_ids.count)
                    new_body_ids_for_skeleton_idx.append([b_id] * len(b_ids_s_i))
                else: 
                    new_body_ids_for_skeleton_idx.append(b_ids_s_i)
            else: 
                new_body_ids_for_skeleton_idx.append(b_ids_s_i)
        body_ids_for_skeleton_idx = new_body_ids_for_skeleton_idx
                
        body_ids_skeleton_idx_cost_matrix = [[-body_ids_of_skeleton.count(body_id) for body_id in body_ids] for body_ids_of_skeleton in body_ids_for_skeleton_idx]
        row_ind, col_ind = linear_sum_assignment(body_ids_skeleton_idx_cost_matrix)
        # body_id_for_skeleton = [""] * row_ind.shape[0]

        merged_map = {}
        for i in range(row_ind.shape[0]):
            merged_map[row_ind[i]] = body_ids[col_ind[i]]

        return merged_map

    def run(self, skeletons, mode="linear_sum_assign"):
        skeletons = Skeleton.remove_empty(skeletons)
        if np.isnan(skeletons).all() or len(skeletons) == 0:
            return self
        if self.history is None or self.history == {}:
            self.history = {"h" + str(i): [e] for i, e in enumerate(skeletons)}
            return self
        if mode == "linear_sum_assign":
            if skeletons.shape[0] == 0:
                body_ids_to_none = set(self.history.keys())
                for body_id in body_ids_to_none:
                    if self.queue_size() >= MAX_QUEUE_SIZE:
                        self.history[body_id].pop(0)
                    self.history[body_id].append(None)
                return self
            skeleton_map = self.get_skeleton_history_map(skeletons)

            curr_body_ids = list(skeleton_map.values())

            # Manage update
            for s_idx in skeleton_map:
                if skeleton_map[s_idx] not in list(self.history.keys()):
                    self.history[skeleton_map[s_idx]] = [None] * self.queue_size()
                if self.queue_size() >= MAX_QUEUE_SIZE:
                    self.history[skeleton_map[s_idx]].pop(0)
                self.history[skeleton_map[s_idx]].append(skeletons[s_idx])

            # Manage not tracked
            body_ids_to_none = set(self.history.keys()).difference(set(curr_body_ids)).copy()
            for body_id in body_ids_to_none:
                if self.queue_size() >= MAX_QUEUE_SIZE:
                    self.history[body_id].pop(0)
                self.history[body_id].append(None)

            # Manage delete
            if self.queue_size() >= MAX_QUEUE_SIZE:
                body_id_to_remove = []
                for body_id in self.history:
                    
                    # Check None in all elements.
                    all_none = True
                    for e in self.history[body_id]:
                        if e is not None:
                            all_none = False 

                    if all_none: 
                        body_id_to_remove.append(body_id)
                
                if len(body_id_to_remove) > 0:
                    for body_id in body_id_to_remove:
                        self.history.pop(body_id, None)

            return self
        elif mode == "argmin":
            raise ValueError("argmin mode not implemented")
        else:
            raise ValueError("mode string not recognized")

    def get_history(self):
        if self.history is None:
            return []

        count_none_for_queue = [(body_id, len([1 for _ in self.history[body_id] if self.history[body_id] is None])) for body_id in self.history]
        body_ids_to_take = [body_id for body_id, count_none in count_none_for_queue if count_none <= COUNT_INVALID]

        data = [
            {
                #"keypoints": [self.history[body_id][t] for t in range(self.queue_size()) if self.history[body_id][t] is not None],
                "keypoints": np.array(self.history[body_id][-1]) if self.history[body_id][-1] is not None else [],
                "body_id": body_id
            } for body_id in body_ids_to_take if body_id in self.history
        ]
        # for e in data:
        #     e["keypoints"] = np.array(e["keypoints"][-1]).tolist() if len(e["keypoints"]) > 0 else []
        return data

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
