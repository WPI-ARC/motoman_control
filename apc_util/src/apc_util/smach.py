import rospy
import traceback


class OnException(object):
    def __init__(self, cb, failure_state):
        self._cb = cb
        self._failure_state = failure_state

    def __call__(self, *args, **kwargs):
        try:
            return self._cb(*args, **kwargs)
        except Exception as e:
            rospy.logerr("Exception occurred while executing state.")
            traceback.print_tb(e.__traceback__)
            return self._failure_state

on_exception = OnException
