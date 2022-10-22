#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################
"""A interface NodeMonitorObserverInterface"""
from typing import Set
from node_monitor import NodeMonitor


class NodeMonitorObserverInterface(object):
    """
    Interface for observers of NodeMonitor
    """

    def on_dead_node_update(self, node_monitor: NodeMonitor, dead_nodes: Set[str]) -> None:
        """
        Callback function when newly dead node is encountered first time

        Args:
            node_monitor (NodeMonitor): Instance of NodeMonitor class
            dead_nodes (Set[str]): Set of all the dead nodes
        """
        pass

    def on_running_node_update(self, node_monitor: NodeMonitor, running_nodes: Set[str]) -> None:
        """
        Callback function when newly running node is encountered first time

        Args:
            node_monitor (NodeMonitor): Instance of NodeMonitor class
            running_nodes (Set[str]): Set of all the dead nodes
        """
        pass

    def on_start(self, node_monitor: NodeMonitor) -> None:
        """
        Callback function when node monitoring is started

        Args:
            node_monitor (NodeMonitor): Instance of NodeMonitor class
        """
        pass

    def on_stop(self, node_monitor: NodeMonitor) -> None:
        """
        Callback function when node monitoring is stopped

        Args:
            node_monitor (NodeMonitor): Instance of NodeMonitor class
        """
        pass

    def on_no_status_change(self, node_monitor: NodeMonitor) -> None:
        """
        Callback function when node monitoring did not see any status change

        Args:
            node_monitor (NodeMonitor): Instance of NodeMonitor class
        """
        pass
