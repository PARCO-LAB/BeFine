"""

"""

# matlab:openExample('vision/MotionBasedMultiObjectTrackingExample')
import sys
import numpy as np
from scipy.spatial import distance_matrix
from scipy.optimize import linear_sum_assignment
sys.path.append('skeleton')
from skeleton import Skeleton

from pykalman import KalmanFilter

# Cost of non assigning a person
COST_OF_NON_ASSIGNMENT = 1
# if not visible for X instant, delete the track
THRESHOLD_INVISIBLE_FOR_TOO_LONG = 20
# activate heuristic only after X frames
THRESHOLD_AGE = 8
# 
TRESHOLD_VISIBLE = 0.6

# # Kalman Filter parameters
# STATE_COVARIANCE_NOISE = [100, 25]
# PROCESS_NOISE = [200, 50]
        

class Track():

    def __repr__(self ):
        return self.id

    def __init__(self, new_id, centroid):
        self.id = "h" + str(new_id)
        # self.kalmanFilter = KalmanFilter(transition_matrices = [
        #     [1, 1, 0, 0],
        #     [0, 1, 0, 0]
        #     [0, 0, 1, 1],
        #     [0, 0, 0, 1]
        # ],observation_matrices=[
        #     [1, 0, 0, 0],
        #     [0, 0, 1, 0]
        # ],transition_covariance=[
        #     [STATE_COVARIANCE_NOISE[0], 0, 0, 0],
        #     [0, STATE_COVARIANCE_NOISE[1], 0, 0]
        #     [0, 0, STATE_COVARIANCE_NOISE[0], 0],
        #     [0, 0, 0, STATE_COVARIANCE_NOISE[1]]
        # ],observation_covariance=[
        #     [PROCESS_NOISE[0], 0, 0, 0],
        #     [0, PROCESS_NOISE[1], 0, 0]
        #     [0, 0, PROCESS_NOISE[0], 0],
        #     [0, 0, 0, PROCESS_NOISE[1]]
        # ])

        self.age = 1
        self.totalVisibleCount = 1
        self.consecutiveInvisibleCount = 0
        self.centroid = centroid

        self.skeleton_history = []

        self.history_state_means = []
        self.history_state_covs = []

class TemporalFusion:
    def __init__(self):
        self.history = None
        self.tracks = []
        self.centroids = []

    def get_used_ids(self):
        return [int(e.id[1:]) for e in self.tracks]

    def get_available_id(self):
        used_ids = self.get_used_ids()
        i = 0
        while True:
            if i not in used_ids:
                return i
            i += 1

    def predictNewLocationsOfTracks(self):
        for t in self.tracks:
            pass
            # TODO: use kalman
            # # Predict the current location of the track.
            # predictedCentroid =t.kalmanFilter.filter_update(
            #     # t.history_state_means[idx-1], 
            #     # t.history_state_covs[idx-1], 
            #     # observation = df.iloc[idx].DOG, 
            #     # observation_matrix=obs_mat
            # )
            # t.centroid = predictedCentroid

    # variant of linear sum assignment
    def assignDetectionsToTracks(self, cost, costOfNonAssignment):
        # pad the matrix to get the "non-assignment metric"
        rows = cost.shape[0]
        cols = cost.shape[1] 

        max_size = rows + cols
        paddedCost = np.zeros((max_size, max_size))
        #paddedCost = np.array((max_size,max_size))
        paddedCost[rows:max_size,0:max_size] = costOfNonAssignment
        paddedCost[0:max_size,cols:max_size] = costOfNonAssignment
        paddedCost[rows:max_size,cols:max_size] = 0
        paddedCost[0:rows,0:cols] = cost

        row_ind, col_ind = linear_sum_assignment(paddedCost)

        unmatchedTracks = []
        unmatchedDetections = []
        matches = []

        for r, c in zip(row_ind, col_ind):
            if r < rows and c >= cols:
                unmatchedTracks.append(r)
            if c < cols and r >= rows:
                unmatchedDetections.append(c)
            if c < cols and r < rows:
                matches.append((r,c))
        return (matches, unmatchedTracks, unmatchedDetections)

    def detectionToTrackAssignment(self, centroids):
        nTracks = len(self.tracks)
        nDetections = len(centroids)
        
        # Compute the cost of assigning each detection to each track.
        cost = np.zeros((nTracks, nDetections))
        for i in range(nTracks):
            for j in range(nDetections):
                # cost[i][j] = distance(self.tracks[i].centroid, self.centroids[j])
                cost[i][j] = np.linalg.norm(np.array(self.tracks[i].centroid)-np.array(centroids[j]))
        
        # Solve the assignment problem.
        costOfNonAssignment = COST_OF_NON_ASSIGNMENT
        assignments, unassignedTracks, unassignedDetections = self.assignDetectionsToTracks(cost, costOfNonAssignment)
        return (assignments, unassignedTracks, unassignedDetections)
    
    def updateAssignedTracks(self, centroids, assignments, skeletons):
        numAssignedTracks = len(assignments)
        for i in range(numAssignedTracks):
            trackIdx = assignments[i][0]
            detectionIdx = assignments[i][1]
            centroid = centroids[detectionIdx]

            track = self.tracks[trackIdx]
            
            # TODO: use kalman
            track.centroid = centroid
            # TODO update history skeleton
            track.skeleton_history.append(skeletons[detectionIdx])
            # # update kalman filter (and relative mean)
            # history_state_means, history_state_covs = track.kalmanFilter.filter_update(
            #     track.history_state_means[-1], 
            #     track.history_state_covs[-1], 
            #     observation = centroid)
            # track.history_state_means.append(history_state_means)
            # track.history_state_covs.append(history_state_covs)
            
            track.age = track.age + 1
            
            # Update visibility.
            track.totalVisibleCount = track.totalVisibleCount + 1
            track.consecutiveInvisibleCount = 0

    def updateUnassignedTracks(self, unassignedTracks):
        for i in range(len(unassignedTracks)):
            ind = unassignedTracks[i]
            self.tracks[ind].age = self.tracks[ind].age + 1
            self.tracks[ind].consecutiveInvisibleCount = self.tracks[ind].consecutiveInvisibleCount + 1
            # TODO update history skeleton
            self.tracks[ind].skeleton_history.append(np.array([]))
    
    def deleteLostTracks(self):
        if len(self.tracks) == 0:
            return

        tracks_to_remove = []
        for t in self.tracks:
            # Compute the fraction of the track's age for which it was visible.
            ages = t.age
            totalVisibleCounts = t.totalVisibleCount
            visibility = totalVisibleCounts / ages;
        
            # Find the indices of 'lost' tracks.
            to_delete = (ages < THRESHOLD_AGE and visibility < TRESHOLD_VISIBLE) or t.consecutiveInvisibleCount >= THRESHOLD_INVISIBLE_FOR_TOO_LONG
            # Delete lost tracks.
            if to_delete:
                tracks_to_remove.append(t)
        # remove from tracks
        for t in tracks_to_remove:
            self.tracks.remove(t)
    def createNewTracks(self, centroids, unassignedDetections, skeletons):
        for i in range(len(unassignedDetections)):
            centroid = centroids[i]
            newTrack = Track(self.get_available_id(), centroid)
            # TODO update history skeleton
            newTrack.skeleton_history.append(skeletons[i])
            self.tracks.append(newTrack)

    def run(self, skeletons, mode="linear_sum_assign"):
        skeletons = Skeleton.remove_empty(skeletons)
        if np.isnan(skeletons).all() or len(skeletons) == 0:
            return self
        # if self.history is None:
        #     self.history = {"h" + str(i): e for i, e in enumerate(skeletons)}
        #     return self

        # get all the medium point for each detection
        centroids = Skeleton.center(skeletons)
        [assignments, unassignedTracks, unassignedDetections
            ] = self.detectionToTrackAssignment(centroids)
        # existing track, associated skeleton
        self.updateAssignedTracks(centroids, assignments, skeletons)
        # existing track, NON-associated skeleton
        self.updateUnassignedTracks(unassignedTracks)
        # criteria to stop tracking people
        self.deleteLostTracks()
        # criteria to start tracking people
        self.createNewTracks(centroids, unassignedDetections, skeletons)

        # TODO: use the above data to create history, something like the following
        # for t in self.tracks: ........

        return self
       
    def get_history(self):
        # return [{"keypoints": self.history[body_id].tolist(), "body_id": body_id} 
        #     for body_id in self.history] if self.history else []
        return [{"keypoints": t.skeleton_history[-1].tolist(), "body_id": t.id} 
            for t in self.tracks]


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
    print("Step 1: ", tf.run(skeletons_np).tracks)
    print("Step 2: ", tf.run(np.array([skeletons_np[0]])).tracks)
    print("Step 3: ", tf.run(np.array([skeletons_np[1]])).tracks)
    print("Step 4: ", tf.run(np.array([skeletons_np[2]])).tracks)
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
    print("Step 5: ", tf.run(skeletons_np).tracks)


if __name__ == '__main__':
    test()
