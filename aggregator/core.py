"""

"""

import time
import os
import json
import numpy as np
import logging

from skeleton import Skeleton
from fusion import SpatialFusion, TemporalFusion
from util import Log, time_ns
from util import GracefulKiller
from util import KEY_MESSAGE, KEY_SKELETON, RESULTS_FOLDER, KEY_FRAME_ID

log = Log(__name__, enable_console=True, enable_file=False)

FILE_STATS_OUT_FPS = os.path.join(RESULTS_FOLDER, "aggregation_fps.csv")
FILE_STATS_NET_LOSS = os.path.join(RESULTS_FOLDER, "net_loss.csv")
FILE_STATS_OUT_DATA = os.path.join(RESULTS_FOLDER, "data.json")

if not os.path.exists(RESULTS_FOLDER):
    os.mkdir(RESULTS_FOLDER)


class Aggregator:
    def __init__(self, subscriber, step=None, window=None, single_person=False, publisher=None, debug=True, log_lvl=logging.DEBUG):
        self.subscriber = subscriber
        self.publisher = publisher
        self.step = step or 60.0
        self.e = (window or self.step) * 0.5
        self.debug = debug
        self.data = []
        self.fusion = TemporalFusion()
        self.aggregated_pdu = 0
        self.single_person = single_person

        if self.debug:
            open(FILE_STATS_OUT_FPS, "w").close()

        log.set_level(log_lvl)

    def loop(self, simulate=True):
        while self.subscriber.last_msg_timestamp == -1:
            time.sleep(0.5)
        curr_time = self.subscriber.last_msg_timestamp

        exit_i = 0
        killer = GracefulKiller()
        while not killer.kill_now:
            t = time_ns()
            curr_time_data = self.subscriber.get_skeletons_interval(curr_time, self.e)
            self.aggregated_pdu += len(curr_time_data)
            timestamps = self.get_timestamps(curr_time_data)
            frame_ids = self.get_frame_ids(curr_time_data)
            skeletons = self.get_skeletons(curr_time_data)
            log.i("==== {} frames ==== : Aggregation time step {} extracted: {}".format(
                  len(timestamps), curr_time, timestamps))

            # Manage empty skeletons
            if not timestamps or not skeletons:
                log.d("Found empty timestamps ({}) or empty skeletons ({})".format(timestamps, skeletons))
                curr_data = {KEY_MESSAGE: curr_time, "con": [], "frame_ids": frame_ids}
                if self.publisher:
                    log.d("Publish data in {}".format(self.publisher.publish_info()))
                    self.publisher.publish(curr_data)

                next_timestamps = self.subscriber.get_first_timestamps()
                next_timestamps = [t for t in next_timestamps if t >= curr_time]
                if len(next_timestamps) > 0:
                    curr_time = min(next_timestamps) + self.e
                else:
                    if simulate:
                        time_to_sleep = max(5, self.step - ((time_ns() - t) * 1e-6))
                        log.d("Sleep for {} ms".format(time_to_sleep))
                        time.sleep(time_to_sleep / 1000.0)
                
                if self.debug:
                    if exit_i > int(10 * 1000 / self.step):
                        log.w("Exit for amount of empty frames > {}".format(int(10 * 1000 / self.step))) 
                        break
                    exit_i += 1
                continue
                
            if self.debug:
                curr_time = max(timestamps) + self.e + 1
            else:
                curr_times = self.subscriber.get_last_timestamps()
                if len(curr_times) > 0:
                    curr_time = min(curr_times)
                else:
                    curr_time = max(timestamps) + self.e + 1
                self.step = self.subscriber.last_latency

            skeletons_np = Skeleton.to_np(skeletons)
            camera_positions = np.array(list(self.subscriber.camera_positions.values()))
            if self.single_person:
                fusion = [Skeleton.fusion(skeletons_np).tolist()]
            else:
                c, spatial_fusion = SpatialFusion.run(skeletons_np, camera_positions, mode='dbscan')
                temporal_fusion = self.fusion.run(spatial_fusion).get_history()
                fusion = temporal_fusion

            curr_data = {KEY_MESSAGE: max(timestamps), "con": fusion, "frame_ids": frame_ids}
            if self.debug:
                self.data.append(curr_data)
            if self.publisher:
                log.d("Publish data in {}".format(self.publisher.publish_info()))
                # print(skeletons)
                # print(curr_data)
                self.publisher.publish(curr_data)

            fps = 1e9 / (time_ns() - t)
            log.i("Aggregation time step {} - fps: {};".format(curr_time, fps))
            if self.debug:
                f = open(FILE_STATS_OUT_FPS, "a")
                f.write("{}\n".format(fps))
                f.close()
            exit_i = 0
            if simulate:
                time_to_sleep = max(5, self.step - ((time_ns() - t) * 1e-6))
                log.d("Sleep for {} ms".format(time_to_sleep))
                time.sleep(time_to_sleep / 1000.0)

        if self.debug:
            with open(FILE_STATS_OUT_DATA, "w") as fd:
                json.dump(self.data, fd)

            with open(FILE_STATS_NET_LOSS, "w") as fd:
                tot_received = 0
                for i, q_received in enumerate(self.subscriber.received_pdus):
                    fd.write("q{}_received_pdu,{}\n".format(i, q_received))
                    tot_received += q_received
                fd.write("aggregated_pdu,{}\n".format(self.aggregated_pdu))
                fd.write("perc_aggregated,{}\n".format(self.aggregated_pdu / tot_received))

    def get_timestamps(self, data):
        return [i[KEY_MESSAGE] for i in data if KEY_MESSAGE in i]

    def get_frame_ids(self, data):
        return [i[KEY_FRAME_ID] for i in data if KEY_FRAME_ID in i]

    def get_skeletons(self, data):
        if self.single_person:
            list_of_skeletons = [i[KEY_SKELETON][0] for i in data if KEY_SKELETON in i and len(i[KEY_SKELETON]) > 0]
        else:
            list_of_skeletons = [skeleton for i in data if KEY_SKELETON in i for skeleton in i[KEY_SKELETON]]
        filtered_list_of_skeletons = [[coords if coords and None not in coords else [] for coords in skeleton]
                                      for skeleton in list_of_skeletons if skeleton]
        return filtered_list_of_skeletons

