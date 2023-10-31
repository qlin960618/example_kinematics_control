import time
######################################
# Setup logger
import logging

try:
    import rospy

    # Custom logging handler that forwards log messages to rospy logger
    class RospyLoggingHandler(logging.Handler):
        def emit(self, record):
            msg = self.format(record)
            if record.levelno >= logging.ERROR:
                rospy.logerr(msg)
            elif record.levelno >= logging.WARNING:
                rospy.logwarn(msg)
            elif record.levelno >= logging.INFO:
                rospy.loginfo(msg)
            else:
                rospy.logdebug(msg)


    logger = logging.getLogger(__name__)
    logger.setLevel(logging.DEBUG)
    logger.addHandler(RospyLoggingHandler())
    # use rospy Rate implimentation
    UNDER_ROS = True

    def get_time():
        return rospy.Time.now().to_sec()

except ModuleNotFoundError:
    import time

    logging.basicConfig()
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.INFO)
    # use time.sleep() implementation
    UNDER_ROS = False


    def get_time():
        return time.perf_counter()


class RateController:
    def __init__(self, loop_rate, debug=False):
        self.ros_rate = None
        self.loop_time = None
        self.target_looprate = None
        self.set_looprate(loop_rate)
        self.debug = debug
        if debug:
            logger.setLevel(logging.DEBUG)

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
        if UNDER_ROS:
            self.ros_rate = rospy.Rate(self.target_looprate)

    def get_target_looprate(self):
        return self.target_looprate

    def get_average_looprate(self):
        return self.avg_looprate

    def get_current_looprate(self):
        return self.current_looprate

    def sleep(self):
        assert self.start_time is not None, "Controller not yet initialized"

        if UNDER_ROS:
            self.ros_rate.sleep()
        else:
            end_time = get_time()
            elapsed_time = end_time - self.start_time
            if elapsed_time < self.loop_time:
                time.sleep(self.loop_time - elapsed_time)
            else:
                if self.debug:
                    logger.debug('target loop rate not achieved:{}'.format(self.current_looprate))

        self.last_start_time = self.start_time
        self.start_time = get_time()
        # Calculate running average of current lo0p rate with 0.1 weight on current
        self.current_looprate = 1.0 / (self.start_time - self.last_start_time)
        self.avg_looprate = self.avg_looprate * 0.9 + self.current_looprate * 0.1
        # logger.info('{}, {}, {}'.format(self.current_looprate, self.loop_time, self.loop_time - elapsed_time))

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
        # logger.info('{}, {}, {}'.format(self.current_looprate, self.loop_time, self.loop_time - elapsed_time))
