import rospy
import traceback


class OnException(object):
    def __init__(self, failure_state):
        self._failure_state = failure_state

    def __call__(self, f):
        def wrapped_f(*args, **kwargs):
            try:
                return f(*args, **kwargs)
            except Exception, err:
                rospy.logerr("Exception occurred while executing state: %s." % str(err))
                traceback.print_last()
                return self._failure_state
        return wrapped_f

on_exception = OnException
