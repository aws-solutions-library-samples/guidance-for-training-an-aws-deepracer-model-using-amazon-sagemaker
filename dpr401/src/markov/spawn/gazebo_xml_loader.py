"""this model handles sdf, urdf, and xacro xml file parsing"""

import os
import xacro

from markov.log_handler.exception_handler import log_and_exit
from markov.log_handler.constants import (SIMAPP_SIMULATION_WORKER_EXCEPTION,
                                          SIMAPP_EVENT_ERROR_CODE_500)


class GazeboXmlLoader(object):
    """GazeboXmlLoader to parse sdf, urdf, and xacro file"""
    @staticmethod
    def parse(file_path: str, **kwargs) -> str:
        """parse sdf, urdf, or xacro file

        Args:
            file_path (str): file path to parse
            **kwargs: Arbitrary keyword arguments

        Returns:
            str: string of processed file contents

        Raises:
            Exception: GazeboXmlLoader parse file loading or non-recognized type
            exception
        """
        _, file_extension = os.path.splitext(file_path)
        try:
            if file_extension in ['.sdf', '.urdf']:
                with open(file_path, "r") as file_pointer:
                    xml = file_pointer.read()
                return xml
            if file_extension == '.xacro':
                xacro_xml = xacro.process_file(input_file_name=file_path,
                                               mappings=kwargs).toxml()
                return xacro_xml
            log_and_exit("[GazeboXmlLoader]: file type {} not recognizable".format(file_extension),
                         SIMAPP_SIMULATION_WORKER_EXCEPTION,
                         SIMAPP_EVENT_ERROR_CODE_500)
        except Exception as ex:
            log_and_exit("[GazeboXmlLoader]: file open or parse failure, {}".format(ex),
                         SIMAPP_SIMULATION_WORKER_EXCEPTION,
                         SIMAPP_EVENT_ERROR_CODE_500)
