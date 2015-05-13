import rospy

from apc_msgs.msg import ProcessedObject, SampleArray
from apc_vision.srv import TakeSample, GetSamples, ProcessSamples, TakeSampleResponse, GetSamplesResponse, ProcessSamplesResponse

_take_sample = rospy.ServiceProxy("take_sample", TakeSample)
_get_samples = rospy.ServiceProxy("get_samples", GetSamples)
_process_samples = rospy.ServiceProxy("process_samples", ProcessSamples)


def take_sample(command, bin):
    for i in range(5):
        try:
            result = _take_sample(command, bin)
            if result.status != TakeSampleResponse.SUCCESS:
                rospy.logwarn("Failure with take_sample(%s, %s): status=%s" % (command, bin, result.status))
            return result.status == TakeSampleResponse.SUCCESS
        except rospy.ServiceException as e:
            rospy.logwarn("Failure with take_sample(%s, %s): %s" % (command, bin, str(e)))
    rospy.logerr("Failed to take sample")
    return False


def get_samples(bin):
    for i in range(5):
        try:
            result = _get_samples(bin)
            if result.status != GetSamplesResponse.SUCCESS:
                rospy.logwarn("Failure with get_samples(%s): status=%s" % (bin, result.status))
            return result.samples, result.status == GetSamplesResponse.SUCCESS
        except rospy.ServiceException as e:
            rospy.logwarn("Failure with get_samples(%s): %s" % (bin, str(e)))
    rospy.logerr("Failed to get samples")
    return None, False


def process_samples(samples, order):
    for i in range(5):
        try:
            result = _process_samples(SampleArray(samples, order))
            if result.result.status != ProcessedObject.SUCCESS:
                rospy.logwarn("Failure with get_samples(<<samplse>>, %s): status=%s" % (order))
            return result, result.result.status == ProcessedObject.SUCCESS
        except rospy.ServiceException as e:
            rospy.logwarn("Failure with get_samples(<<samples>>, %s): %s" % (order, str(e)))
    rospy.logerr("Failed to process samples")
    return None, False
