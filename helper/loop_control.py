import time
######################################
import time


def get_time():
    return time.perf_counter()


class RateController:
    def __init__(self, loop_rate, debug=False):
        self.ros_rate = None
        self.loop_time = None
        self.target_looprate = None
        self.set_looprate(loop_rate)
        self.debug = debug

        self.current_looprate = 0
        self.avg_looprate = 0

        self.start_time = None
        self.end_time = None
        self.last_start_time = None

    def initialize(self):
        self.start_time = get_time()
        self.last_start_time = self.start_time

    def set_looprate(self, rate):
        assert rate > 0, "rate has to be positive"
        self.target_looprate = rate
        self.loop_time = 1.0 / rate

    def get_target_looprate(self):
        return self.target_looprate

    def get_average_looprate(self):
        return self.avg_looprate

    def get_current_looprate(self):
        return self.current_looprate

    def sleep(self):
        assert self.start_time is not None, "Controller not yet initialized"

        end_time = get_time()
        elapsed_time = end_time - self.start_time
        if elapsed_time < self.loop_time:
            time.sleep(self.loop_time - elapsed_time)
        else:
            pass  # loop is running slower than target

        self.last_start_time = self.start_time
        self.start_time = get_time()
        # Calculate running average of current lo0p rate with 0.1 weight on current
        self.current_looprate = 1.0 / (self.start_time - self.last_start_time)
        self.avg_looprate = self.avg_looprate * 0.9 + self.current_looprate * 0.1

    def fast_sleep(self):
        """
        sleep function for simulation, without actually sleeping
        Returns:
        """
        assert self.start_time is not None, "Controller not yet initialized"

        self.last_start_time = self.start_time
        self.start_time = get_time()
        # Calculate running average of current lo0p rate with 0.1 weight on current
        self.current_looprate = 1.0 / (self.start_time - self.last_start_time)
        self.avg_looprate = self.avg_looprate * 0.9 + self.current_looprate * 0.1
