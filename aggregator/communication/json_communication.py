"""

"""

import os
import json

from util import Log
from util import time_ns
from util.constant import *

log = Log(__name__, enable_console=True, enable_file=False)


class JsonCommunication:
    def __init__(self, topics, files, analysis=True):
        self.topics = {}
        self.queues = []
        self.received_pdus = []
        for i, (t, _) in enumerate(topics):
            self.topics[t] = i
            self.queues.append([])
            self.received_pdus.append(0)

        for i, f in enumerate(files):
            with open(f, 'r') as fd:
                j = json.load(fd)
            self.queues[i] = j
            self.received_pdus[i] = len([e for e in j if KEY_MESSAGE in e])
        
        self.analysis = analysis
        self.last_msg_timestamp = -1

    def start(self):
        self.last_msg_timestamp = min([q[0][KEY_MESSAGE] for q in self.queues if len(q) > 0])

    def get_skeletons_interval(self, time, error):
        ret = []
        for q_i, q in enumerate(self.queues):
            q_ret = [i for i in q
                     if KEY_MESSAGE in i and time - error <= i[KEY_MESSAGE] <= time + error]
            if q_ret:
                discarded_amount = (q.index(q_ret[-1]) + 1) - len(q_ret)
                if discarded_amount > 0:
                    timestamp_discarded = [e[KEY_MESSAGE] for e in q[:discarded_amount]]
                    log.w("Frames discarded {} from queue of topic {}".format(timestamp_discarded, list(self.topics.keys())[q_i]))
                ret.extend(q_ret)
                del q[:q.index(q_ret[-1]) + 1]
        return ret

    def get_first_timestamps(self):
        return [q[0][KEY_MESSAGE] for q in self.queues if q and KEY_MESSAGE in q[0]]

    def publish_info(self):
        return ""
