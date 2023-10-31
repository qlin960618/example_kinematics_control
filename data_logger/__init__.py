import scipy
import scipy.io as sio
import dqrobotics as dql
import numpy as np
import os
import datetime
import logging

# logger init
logging.basicConfig()
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

class DataLogger:
    def __init__(self, save_prefix):
        self.save_prefix = save_prefix

        # prepare directory
        os.makedirs(self.save_prefix, exist_ok=True)

        # data object
        self.log_buffer = {}

    def log(self, key, data):
        """
        Wrapper for unifying data type
        Args:
            key (string): name of the key
            data: data input
        Returns:
            bool: The data is logged successfully
        """
        if isinstance(data, dql.DQ):
            _data = data.vec8()
        elif isinstance(data, (float, int)):
            _data = np.array([data])
        elif isinstance(data, np.ndarray):
            _data = data
            while len(_data.shape) > 1:
                _data = _data.reshape(-1)
        else:
            raise ValueError("Input value is not accepted")

        return self._log_entry(key, _data)

    def _log_entry(self, key, data):
        # check if key in buffer
        if key not in self.log_buffer:
            self.log_buffer[key] = np.vstack([data])
        else:
            if data.shape[0] == self.log_buffer[key].shape[1]:
                self.log_buffer[key] = np.vstack([self.log_buffer[key], data])
            else:
                raise ValueError("input array shape mismatched")
        return True

    def save(self, overwrite_name=None):
        if overwrite_name is None:
            save_ = os.path.join(self.save_prefix,
                                 '{date:%Y_%m_%d_%H_%M_%S}.mat'.
                                 format(date=datetime.datetime.now()))
        else:
            save_ = os.path.join(self.save_prefix, overwrite_name)

        sio.savemat(save_, self.log_buffer)
        logger.info("save done")


if __name__ == '__main__':
    log_h = DataLogger("./logs")
    log_h.log('aa1', dql.DQ([1,0,0,0]))
    log_h.log('aqwe', dql.DQ([1,1,0,0]))
    log_h.log('aa1', dql.DQ([1,1,0,0]))
    log_h.log('aqwe', dql.DQ([1,1,0,0]))
    log_h.log('aa1', dql.DQ([1,2,0,0]))
    log_h.log('aqwe', dql.DQ([1,1,0,0]))
    log_h.log('aa1', dql.DQ([1,1,0,0]))

    log_h.save()
