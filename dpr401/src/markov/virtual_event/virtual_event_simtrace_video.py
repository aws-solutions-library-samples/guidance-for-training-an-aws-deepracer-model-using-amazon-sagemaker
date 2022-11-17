import rospy

from deepracer_simulation_environment.srv import (VirtualEventVideoEditSrv,
                                                  VirtualEventVideoEditSrvRequest)
from markov.boto.s3.constants import (CAMERA_45DEGREE_LOCAL_PATH_FORMAT,
                                      CAMERA_PIP_MP4_LOCAL_PATH_FORMAT,
                                      CAMERA_TOPVIEW_LOCAL_PATH_FORMAT,
                                      SIMTRACE_EVAL_LOCAL_PATH_FORMAT,
                                      SimtraceVideoNames)
from markov.boto.s3.files.simtrace_video import SimtraceVideo
from markov.constants import DEFAULT_COLOR
from markov.utils import get_s3_extra_args
from markov.rospy_wrappers import ServiceProxyWrapper
from std_srvs.srv import (Empty,
                          EmptyRequest)


class VirtualEventSimtraceVideo():
    """
    VirtualEventSimtraceVideo class
    """
    def __init__(self, profile, region):
        """
        VirtualEventSimtraceVideo constructor

        Args:
            profile (object): racer profile object
            region (str): aws region
        """
        self._is_saving_simtrace = False
        self._is_saving_mp4 = False
        self._region = region
        self._profile = profile
        self._simtrace_video_s3_writers = []
        self._subscribe_to_save_mp4, self._unsubscribe_from_save_mp4 = None, None

        if hasattr(profile, 'outputSimTrace'):
            self._simtrace_video_s3_writers.append(
                SimtraceVideo(upload_type=SimtraceVideoNames.SIMTRACE_EVAL.value,
                              bucket=profile.outputSimTrace.s3BucketName,
                              s3_prefix=profile.outputSimTrace.s3KeyPrefix,
                              region_name=self._region,
                              local_path=SIMTRACE_EVAL_LOCAL_PATH_FORMAT.format(profile.agent_name)))
            self._is_saving_simtrace = True
        if hasattr(profile, 'outputMp4'):
            self._simtrace_video_s3_writers.extend([
                SimtraceVideo(upload_type=SimtraceVideoNames.PIP.value,
                              bucket=profile.outputMp4.s3BucketName,
                              s3_prefix=profile.outputMp4.s3KeyPrefix,
                              region_name=self._region,
                              local_path=CAMERA_PIP_MP4_LOCAL_PATH_FORMAT.format(profile.agent_name)),
                SimtraceVideo(upload_type=SimtraceVideoNames.DEGREE45.value,
                              bucket=profile.outputMp4.s3BucketName,
                              s3_prefix=profile.outputMp4.s3KeyPrefix,
                              region_name=self._region,
                              local_path=CAMERA_45DEGREE_LOCAL_PATH_FORMAT.format(profile.agent_name)),
                SimtraceVideo(upload_type=SimtraceVideoNames.TOPVIEW.value,
                              bucket=profile.outputMp4.s3BucketName,
                              s3_prefix=profile.outputMp4.s3KeyPrefix,
                              region_name=self._region,
                              local_path=CAMERA_TOPVIEW_LOCAL_PATH_FORMAT.format(profile.agent_name))])
            self._is_saving_mp4 = True

    @property
    def is_saving_simtrace(self):
        """
        Returns:
            bool: True if saving simtrace, False otherwise
        """
        return self._is_saving_simtrace

    def setup(self):
        """
        Setup simtrace video
        """
        mp4_sub = "/{}/save_mp4/subscribe_to_save_mp4".format(self._profile.racecar_name)
        mp4_unsub = "/{}/save_mp4/unsubscribe_from_save_mp4".format(self._profile.racecar_name)
        rospy.wait_for_service(mp4_sub)
        rospy.wait_for_service(mp4_unsub)
        self._subscribe_to_save_mp4 = ServiceProxyWrapper(mp4_sub, VirtualEventVideoEditSrv)
        self._unsubscribe_from_save_mp4 = ServiceProxyWrapper(mp4_unsub, Empty)
        if self._is_saving_mp4:
            # racecar_color is not used for virtual event image editing, so simply pass default "Black"
            self._subscribe_to_save_mp4(VirtualEventVideoEditSrvRequest(
                display_name=self._profile.racerAlias,
                racecar_color=DEFAULT_COLOR))

    def persist(self):
        """
        Persist simtrace video
        """
        if self._is_saving_mp4:
            self._unsubscribe_from_save_mp4(EmptyRequest())
        if hasattr(self._profile.outputMp4, 's3KmsKeyArn'):
            simtrace_mp4_kms = self._profile.outputMp4.s3KmsKeyArn
        elif hasattr(self._profile.outputSimTrace, 's3KmsKeyArn'):
            simtrace_mp4_kms = self._profile.outputSimTrace.s3KmsKeyArn
        else:
            simtrace_mp4_kms = None
        for s3_writer in self._simtrace_video_s3_writers:
            s3_writer.persist(get_s3_extra_args(simtrace_mp4_kms))
