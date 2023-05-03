"""
"""

import os
import json
import argparse
import configparser
import logging

from core import Aggregator
from communication import MqttCommunication, KafkaCommunication
from util import Log

log = Log(__name__, enable_console=True, enable_file=False)

# Script arguments
parser = argparse.ArgumentParser(description="BeFine", epilog="Mirco De Marchi & Michele Boldo")
parser.add_argument("--log",
                    "-l",
                    dest="log_lvl",
                    required=False,
                    help="log level of the project: DEBUG, INFO, WARNING, ERROR, FATAL",
                    default="DEBUG")
parser.add_argument("--config",
                    "-c",
                    dest="config_file",
                    required=False,
                    help="path to configuration file with all settings",
                    default="defconfig.ini")
parser.add_argument("--subscriber",
                    "-sub",
                    dest="subscriber",
                    required=True,
                    help="A string between: [\"mqtt\", \"kafka\"]")
parser.add_argument("--publisher",
                    "-pub",
                    dest="publisher",
                    required=False,
                    help="A string between: [\"mqtt\", \"kafka\", \"none\"]",
                    default="none")
args = parser.parse_args()


def main():
    # Handle args
    log_lvl = getattr(logging, args.log_lvl.upper(), logging.DEBUG)
    log.set_level(log_lvl)
    config_file = os.path.join(os.path.dirname(__file__), args.config_file)
    if not os.path.exists(config_file):
        log.f("Configuration file do not find, exit")
        return

    # Handle config
    config = configparser.ConfigParser(allow_no_value=True)
    config.read(config_file)

    # Init communication
    if args.subscriber not in ["kafka", "mqtt"]:
        log.e("subscriber argument is not in [\"mqtt\", \"kafka\"]")
        return
    if args.publisher not in ["kafka", "mqtt", "none"]:
        log.e("publisher argument is not in [\"mqtt\", \"kafka\", \"none\"]")
        return 

    subscriber_class, publisher_class = None, None
    if args.subscriber == "kafka":
        subscriber_class = KafkaCommunication
    else: # mqtt
        subscriber_class = MqttCommunication

    if args.publisher == "kafka":
        publisher_class = KafkaCommunication
    elif args.publisher == "mqtt": 
        publisher_class = MqttCommunication

    topics = json.loads(config["SUBSCRIBE-COMMUNICATION"]["topic"])
    communication_topics = [(t, config["SUBSCRIBE-COMMUNICATION"].getint("qos")) for t in topics]
    subscriber = subscriber_class(
        communication_topics,
        config["SUBSCRIBE-COMMUNICATION"]["broker-ip"],
        config["SUBSCRIBE-COMMUNICATION"].getint("port"),
        config["SUBSCRIBE-COMMUNICATION"]["clientid"],
        config["SUBSCRIBE-COMMUNICATION"]["username"],
        config["SUBSCRIBE-COMMUNICATION"]["password"],
        None,
        config["GENERAL"].getboolean("debug"), log_lvl
    )

    publisher = None
    if publisher_class is not None:
        publisher = publisher_class(
            None,
            config["PUBLISH-COMMUNICATION"]["broker-ip"],
            config["PUBLISH-COMMUNICATION"].getint("port"),
            config["PUBLISH-COMMUNICATION"]["clientid"],
            config["PUBLISH-COMMUNICATION"]["username"],
            config["PUBLISH-COMMUNICATION"]["password"][1:-1],
            config["PUBLISH-COMMUNICATION"]["topic"] or "/aggregator",
            config["GENERAL"].getboolean("debug"), log_lvl,
            config["PUBLISH-COMMUNICATION"]["protocol"],
            config["PUBLISH-COMMUNICATION"]["sasl_mechanism"]

        )

    # Init aggregator
    aggregator = Aggregator(
        subscriber,
        config["GENERAL"].getint("step-init") if config["GENERAL"]["step-init"] else None,
        config["GENERAL"].getfloat("window") if config["GENERAL"]["window"] else None,
        False,
        publisher,
        config["GENERAL"].getboolean("debug"), log_lvl
    )

    # Run the system
    subscriber.start()
    aggregator.loop()


if __name__ == '__main__':
    main()