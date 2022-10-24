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
from unittest import TestCase
from unittest.mock import patch, MagicMock

# rosnode is imported in NodeMonitor class, but this package is not a ROS environment.
# Since this is missing the rosnode, mocking the rosnode module.
# https://stackoverflow.com/questions/8658043/how-to-mock-an-import
import sys
sys.modules['rosnode'] = MagicMock()

from node_monitor import NodeMonitor


@patch("node_monitor.node_monitor.RLock")
class NodeMonitorTest(TestCase):
    def setUp(self) -> None:
        self.monitor_nodes = ['/gazebo', '/deepracer', '/ude_ros_server', '*/controller_manager',
                              '/agent0/robot_state_publisher']
        self.update_rate_hz = 0.2

    def test_initialize(self, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        self.assertEqual(node_monitor.monitor_nodes, self.monitor_nodes)
        self.assertEqual(node_monitor.update_rate_hz, self.update_rate_hz)
        self.assertEqual(node_monitor.running_nodes, set())
        self.assertEqual(node_monitor.dead_nodes, set())
        self.assertEqual(rlock_mock.call_count, 3)
        self.assertEqual(node_monitor._observers, set())
        self.assertFalse(node_monitor._is_monitoring)
        self.assertEqual(node_monitor._seen_nodes, set())

    def test_register(self, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        observer1 = MagicMock()
        observer2 = MagicMock()
        node_monitor.register(observer1)
        node_monitor.register(observer2)
        self.assertEqual(node_monitor._observers, set({observer1, observer2}))

    def test_unregister(self, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        observer1 = MagicMock()
        observer2 = MagicMock()
        node_monitor._observer_lock = MagicMock()
        node_monitor._observers = set({observer1, observer2})
        node_monitor.unregister(observer1)
        self.assertEqual(node_monitor._observers, set({observer2}))

    def test_is_monitor_node_true(self, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        self.assertTrue(node_monitor._is_monitor_node('/gazebo'))
        self.assertTrue(node_monitor._is_monitor_node('/agent0/controller_manager'))
        self.assertTrue(node_monitor._is_monitor_node('/abc/controller_manager'))

    def test_is_monitor_node_false(self, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        self.assertFalse(node_monitor._is_monitor_node('/hello'))
        self.assertFalse(node_monitor._is_monitor_node('/controller'))

    @patch("node_monitor.node_monitor.rosnode")
    def test_update_running_nodes_with_newly_updated_running_node(self, rosnode_mock, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        rosnode_mock.rosnode_ping_all.return_value = (['/gazebo'], "dummy_value")
        node_monitor._update_running_nodes()
        rosnode_mock.rosnode_ping_all.assert_called_once()
        self.assertEqual(node_monitor._seen_nodes, set({'/gazebo'}))
        self.assertEqual(node_monitor.running_nodes, set({'/gazebo'}))

    @patch("node_monitor.node_monitor.rosnode")
    def test_update_running_nodes_with_newly_updated_running_node_empty_monitor_node(self, rosnode_mock, rlock_mock):
        node_monitor = NodeMonitor(list(), self.update_rate_hz)
        rosnode_mock.rosnode_ping_all.return_value = (['/gazebo'], "dummy_value")
        node_monitor._update_running_nodes()
        rosnode_mock.rosnode_ping_all.assert_called_once()
        self.assertEqual(node_monitor._seen_nodes, set({'/gazebo'}))
        self.assertEqual(node_monitor.running_nodes, set({'/gazebo'}))

    @patch("node_monitor.node_monitor.rosnode")
    def test_update_running_nodes_with_newly_updated_not_in_monitor(self, rosnode_mock, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        rosnode_mock.rosnode_ping_all.return_value = (['/not_in_monitor'], "dummy_value")
        node_monitor._update_running_nodes()
        rosnode_mock.rosnode_ping_all.assert_called_once()
        self.assertEqual(node_monitor._seen_nodes, set())
        self.assertEqual(node_monitor.running_nodes, set())

    @patch("node_monitor.node_monitor.rosnode")
    def test_update_running_nodes_already_seen_node(self, rosnode_mock, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        rosnode_mock.rosnode_ping_all.return_value = (['/gazebo'], "dummy_value")
        node_monitor._seen_nodes = set({'/gazebo'})
        node_monitor._running_nodes = set({'/gazebo'})
        node_monitor._update_running_nodes()
        rosnode_mock.rosnode_ping_all.assert_called_once()

    def test_update_running_nodes_failed_master_node(self, rlock_mock):
        with self.assertRaises(Exception):
            node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
            node_monitor._update_running_nodes.side_effect = Exception()
            node_monitor._update_running_nodes()
            self.assertEqual(node_monitor._seen_nodes, set())
            self.assertEqual(node_monitor.running_nodes, set())

    @patch("node_monitor.node_monitor.rosnode")
    def test_update_running_nodes_failed_master_node_specific_exception(self, rosnode_mock, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        rosnode_mock.rosnode_ping_all.side_effect = Exception()
        node_monitor._update_running_nodes()
        self.assertEqual(node_monitor._seen_nodes, set())
        self.assertEqual(node_monitor.running_nodes, set())

    def test_update_dead_nodes_success(self, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        node_monitor._seen_nodes = set({'/gazebo', '/deepracer'})
        node_monitor._running_nodes = set()
        node_monitor._dead_nodes = set()
        node_monitor._update_dead_nodes()
        self.assertEqual(node_monitor._dead_nodes, set({'/gazebo', '/deepracer'}))

    def test_update_dead_nodes_already_in_list(self, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        node_monitor._seen_nodes = set({'/gazebo'})
        node_monitor._running_nodes = set()
        node_monitor._dead_nodes = set({'/gazebo'})
        node_monitor._update_dead_nodes()
        self.assertEqual(node_monitor._dead_nodes, set({'/gazebo'}))

    @patch("time.sleep", side_effect=InterruptedError)
    def test_start(self, sleep_mock, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        observer1, observer2 = MagicMock(), MagicMock()
        observer1.on_start = MagicMock()
        observer2.on_start = MagicMock()
        node_monitor._observers = set({observer1, observer2})
        node_monitor._update_running_nodes = MagicMock()
        node_monitor._update_dead_nodes = MagicMock()
        node_monitor.start()
        observer1.on_start.assert_called_once()
        observer2.on_start.assert_called_once()
        node_monitor._update_running_nodes.assert_called_once()
        node_monitor._update_dead_nodes.assert_called_once()
        self.assertTrue(node_monitor._is_monitoring)

    @patch("time.sleep", side_effect=InterruptedError)
    def test_start_calling_running_dead(self, sleep_mock, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        observer1, observer2 = MagicMock(), MagicMock()
        observer1.on_start = MagicMock()
        observer2.on_start = MagicMock()
        observer1.on_running_node_update = MagicMock()
        observer2.on_running_node_update = MagicMock()
        observer1.on_dead_node_update = MagicMock()
        observer2.on_dead_node_update = MagicMock()
        node_monitor._observers = set({observer1, observer2})
        node_monitor._update_running_nodes = MagicMock()
        node_monitor._update_dead_nodes = MagicMock()
        node_monitor._running_nodes = MagicMock()
        node_monitor._dead_nodes = MagicMock()
        node_monitor._running_nodes.copy.return_value = set({'/gazebo'})
        node_monitor._dead_nodes.copy.return_value = set({'/gazebo'})
        node_monitor.start()
        observer1.on_start.assert_called_once()
        observer2.on_start.assert_called_once()
        node_monitor._update_running_nodes.assert_called_once()
        node_monitor._update_dead_nodes.assert_called_once()
        observer1.on_running_node_update.assert_called_once()
        observer2.on_running_node_update.assert_called_once()
        observer1.on_dead_node_update.assert_called_once()
        observer2.on_dead_node_update.assert_called_once()
        self.assertTrue(node_monitor._is_monitoring)

    @patch("time.sleep", side_effect=InterruptedError)
    def test_start_no_status_change_update(self, sleep_mock, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        node_monitor._is_update_observers_on_no_status_change = True
        observer1, observer2 = MagicMock(), MagicMock()
        observer1.on_start = MagicMock()
        observer2.on_start = MagicMock()
        observer1.on_running_node_update = MagicMock()
        observer2.on_running_node_update = MagicMock()
        observer1.on_dead_node_update = MagicMock()
        observer2.on_dead_node_update = MagicMock()
        observer1.on_no_status_change = MagicMock()
        observer2.on_no_status_change = MagicMock()
        node_monitor._observers = set({observer1, observer2})
        node_monitor._update_running_nodes = MagicMock()
        node_monitor._update_dead_nodes = MagicMock()
        node_monitor._running_nodes = set({'/gazebo'})
        node_monitor._dead_nodes = set()
        node_monitor.start()
        observer1.on_start.assert_called_once()
        observer2.on_start.assert_called_once()
        node_monitor._update_running_nodes.assert_called_once()
        node_monitor._update_dead_nodes.assert_called_once()
        observer1.on_running_node_update.assert_not_called()
        observer2.on_running_node_update.assert_not_called()
        observer1.on_no_status_change.assert_called_once()
        observer2.on_no_status_change.assert_called_once()
        self.assertTrue(node_monitor._is_monitoring)

    def test_stop(self, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        observer1, observer2 = MagicMock(), MagicMock()
        observer1.on_stop = MagicMock()
        observer2.on_stop = MagicMock()
        node_monitor._observers = set({observer1, observer2})
        node_monitor.stop()
        observer1.on_stop.assert_called_once()
        observer2.on_stop.assert_called_once()
        self.assertFalse(node_monitor._is_monitoring)
