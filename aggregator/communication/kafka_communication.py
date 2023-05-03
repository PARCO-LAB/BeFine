"""

"""

import os
from json import dumps
from kafka import KafkaProducer
import numpy as np
import logging

JOINTS_FROM_INDEX = [
    "nose",
    "left_ear",
    "right_ear",
    "left_shoulder",
    "right_shoulder",
    "left_elbow",
    "right_elbow",
    "left_wrist",
    "right_wrist",
    "left_hip",
    "right_hip",
    "left_knee",
    "right_knee",
    "left_ankle",
    "right_ankle",
    "neck",
    "chest",
    "mid_hip"
]


class KafkaCommunication:
    def __init__(self, topics, uri, port=9092, client_id="", username="", password="", publish_topic="", analysis=True, log_lvl=logging.DEBUG, protocol = 'SASL_PLAINTEXT', sasl_mechanism='SCRAM-SHA-512'):
        self.topics = topics
        self.publish_topic = publish_topic
        self.bootstrap_servers = [uri + ":" + str(port)]
        # self.producer = KafkaProducer(bootstrap_servers=self.bootstrap_servers, value_serializer=lambda x: dumps(x).encode("utf-8"))
        self.producer = KafkaProducer(bootstrap_servers=self.bootstrap_servers, security_protocol=protocol, sasl_mechanism=sasl_mechanism, sasl_plain_username=username, sasl_plain_password=password, value_serializer=lambda x: dumps(x).encode("utf-8"))

    def format_to_kafka(self,data):

        if False:
            kafka_msg = {
                "timestamp" : data["timestamp"],
                "bodies": [
                    {
                        "body_id": str(body_obj["body_id"]), 
                        "event": [], 
                        "keypoints": 
                        {
                            JOINTS_FROM_INDEX[parts]: [dict(zip(["x", "y", "z"], body_obj["keypoints"][parts]))] 
                                if len(body_obj["keypoints"][parts]) == 3 and not np.isnan(body_obj["keypoints"][parts]).all()
                                else []
                                for parts in range(len(body_obj["keypoints"])) 
                        }
                    } for body_obj in data["con"]
                ]
            }
        
        else:
            kafka_msg = {
                "timestamp" : data["timestamp"],
                "bodies": [
                    {
                        "body_id": str(body_obj["body_id"]), 
                        "event": [], 
                        "keypoints": 
                        {
                            JOINTS_FROM_INDEX[parts]: [dict(zip(["x", "y", "z"], body_obj["keypoints"][parts]))] 
                                if len(body_obj["keypoints"][parts]) == 3 and not np.isnan(body_obj["keypoints"][parts]).all()
                                else [dict(zip(["x", "y", "z"], [None, None, None]))] 
                                for parts in range(len(body_obj["keypoints"])) 
                        }
                    } for body_obj in data["con"]
                ]
            }


        # kafka_msg = {"timestamp" : data["timestamp"]}
        # kafka_msg["body"] = []
        # for body_obj in data["con"]:
        #     body = {"body_id": 1}
        #     body["event"] = []
        #     body["keypoints"] = {}
        #     for parts in range(0,len(body_obj)):
        #         kp_temp = {"x": body_obj[parts][0], "y" : body_obj[parts][1], "z" : body_obj[parts][2]} if len(body_obj[parts]) == 3 else {}
        #         body["keypoints"][JOINTS_FROM_INDEX[parts]] = kp_temp
        #     kafka_msg["body"].append(body)
        return kafka_msg

    def publish(self, data):
        data_processed = self.format_to_kafka(data)
        self.producer.send(self.publish_topic, value=data_processed)
        self.producer.flush() #< Comment for async.

    def publish_info(self):
        return self.publish_topic

