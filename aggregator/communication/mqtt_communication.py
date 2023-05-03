"""

"""

import os
import json
import paho.mqtt.client as mqtt
import logging

from util import Log
from util import time_ns
from util.constant import *
from skeleton import Skeleton

log = Log(__name__, enable_console=True, enable_file=False)

FILE_STATS_NET_FPS = os.path.join(RESULTS_FOLDER, "network_fps.csv")
FILE_STATS_NET_LATENCY = os.path.join(RESULTS_FOLDER, "network_latency.csv")

KNOWN_CAMERA_POSITIONS = {
    "befine/jetsonzed05": [6.95192622706614, -0.307240670798729, 3.12092876437291],
    "befine/jetsonzed07": [0.15295255126497, -3.84592431248964, 2.95945473578165],
    "befine/jetsonzed08": [4.54271195022572, -4.68008583158485, 3.18311133727264],
    "befine/jetsonzed09": [8.42896079710467, -4.56405844269265, 3.09719037563036]
}


class MqttCommunication:
    def __init__(self, topics, uri, port=1883, client_id="", username="", password="", publish_topic="", analysis=True, log_lvl=logging.DEBUG):
        self.mqttc = mqtt.Client(client_id)
        self.mqttc.on_message = self.on_message
        self.mqttc.on_connect = self.on_connect
        self.mqttc.on_publish = self.on_publish
        self.mqttc.on_subscribe = self.on_subscribe
        self.mqttc.on_log = self.on_log
        self.mqttc.username_pw_set(username=username, password=password)
        self.mqttc.connect(uri, port)
        if topics:
            self.mqttc.subscribe(topics)
        self.publish_topic = publish_topic

        self.topics = {}
        self.camera_positions = {}
        self.prev_times = []
        self.queues = []
        self.received_pdus = []
        for i, (t, _) in enumerate(topics or []):
            self.topics[t] = i
            if t in KNOWN_CAMERA_POSITIONS:
                self.camera_positions[t] = KNOWN_CAMERA_POSITIONS[t]
            self.prev_times.append(time_ns())
            self.queues.append([])
            self.received_pdus.append(0)

        self.last_msg_timestamp = -1
        self.last_latency = -1

        self.analysis = analysis
        if self.analysis:
            open(FILE_STATS_NET_LATENCY, "w").close()
            open(FILE_STATS_NET_FPS, "w").close()

        log.set_level(log_lvl)

    def publish(self, msg):
        _ = self.mqttc.publish(self.publish_topic, json.dumps(msg))

    def publish_info(self):
        return self.publish_topic

    def start(self):
        self.mqttc.loop_start()

    def get_skeletons_interval(self, time, error):
        ret = []
        for q in self.queues:
            q_ret = [i for i in q
                     if KEY_MESSAGE in i and time - error <= i[KEY_MESSAGE] <= time + error]
            if q_ret:
                ret.extend(q_ret)
                del q[:q.index(q_ret[-1]) + 1]
        return ret

    def get_first_timestamps(self):
        return [q[0][KEY_MESSAGE] for q in self.queues if q and KEY_MESSAGE in q[0]]

    def get_last_timestamps(self):
        return [q[-1][KEY_MESSAGE] for q in self.queues if q and KEY_MESSAGE in q[-1]]

    def on_connect(self, mqttc, obj, flags, rc):
        log.d("on_connect: {}".format(rc))

    def on_message(self, mqttc, obj, msg):
        topic = msg.topic

        # Decode message
        t = time_ns()
        m_decode = str(msg.payload)
        m_decode = m_decode[2: len(m_decode) - 1]
        m_in = json.loads(m_decode)
        if not m_in or m_in is None or KEY_MESSAGE not in m_in or KEY_PUBLISH not in m_in: 
            log.w("None received from topic {}".format(m_in))
            return
        
        if topic in self.topics:
            topic_idx = self.topics[topic]

            skeletons_np = Skeleton.to_np(self.get_skeletons([m_in]))

            skeletons_filtered = []
            for skeleton in skeletons_np:
                center = Skeleton.center(skeleton)
                dist = Skeleton.dist(center, KNOWN_CAMERA_POSITIONS[topic])

                if dist < 6.0: # meters
                    skeletons_filtered.append(skeleton.tolist())
                
                elif dist < 8.0 and topic != "befine/jetsonzed07": # meters
                    skeletons_filtered.append(skeleton.tolist())
            
            m_in[KEY_SKELETON] = skeletons_filtered
            self.queues[topic_idx].append(m_in)
            self.received_pdus[topic_idx] += 1

            latency = (t - self.prev_times[topic_idx]) * 1e-6
            self.last_latency = latency
            self.last_msg_timestamp = m_in[KEY_MESSAGE]
            if self.analysis:
                fps = 1e9 / (t - (m_in[KEY_PUBLISH] * 1e6))
                log.i("on_message from topic {} - net_inter_latency: {} ms; net_fps: {};".format(topic, latency, fps))
                log.i("on_message from topic {} - timestamp: {} ms;".format(topic, m_in[KEY_MESSAGE]))
                f = open(FILE_STATS_NET_LATENCY, "a")
                f.write("{}\n".format(latency))
                f.close()
                f = open(FILE_STATS_NET_FPS, "a")
                f.write("{}\n".format(fps))
                f.close()
            self.prev_times[topic_idx] = time_ns()
        else:
            log.w("on_message from topic not recognized: {}".format(topic))

    def on_publish(self, mqttc, obj, mid):
        log.d("on_publish: {}".format(mid))

    def on_subscribe(self, mqttc, obj, mid, granted_qos):
        log.d("on_subscribe: {} {}".format(mid, granted_qos))

    def on_log(self, mqttc, obj, level, string):
        log.d("on_log: {}".format(string))


    def get_skeletons(self, data):
       
        list_of_skeletons = [skeleton for i in data if KEY_SKELETON in i for skeleton in i[KEY_SKELETON]]
        filtered_list_of_skeletons = [[coords if coords and None not in coords else [] for coords in skeleton]
                                      for skeleton in list_of_skeletons if skeleton]
        return filtered_list_of_skeletons
