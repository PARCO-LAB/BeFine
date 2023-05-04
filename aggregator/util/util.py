"""
"""

import time
import signal


def get_elapsed(start, end):
    hours, rem = divmod(end - start, 3600)
    minutes, seconds = divmod(rem, 60)
    return "{:0>2}:{:0>2}:{:05.2f}".format(int(hours), int(minutes), seconds)


def time_ns():
    return int(time.time() * 1e9)


def get_elapsed_ns(start, end):
    hours, rem = divmod((end - start), 3600 * 1e9)
    minutes, rem = divmod(rem, 60 * 1e9)
    seconds, rem = divmod(rem, 1e9)
    milli, rem = divmod(rem, 1e6)
    micro, nano = divmod(rem, 1e3)
    return "{:0>2}:{:0>2}:{:0>2} {:0>3}:{:0>3}:{:0>3}".format(
        int(hours), int(minutes), int(seconds), int(milli), int(micro), int(nano))


class GracefulKiller:
    def __init__(self):
        self.kill_now = False
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)

    def exit_gracefully(self, *args):
        self.kill_now = True