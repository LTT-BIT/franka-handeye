"""Real ROS transports with synthetic camera and TF; never connects hardware."""
import time

import cv2
import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy', reason='先 source 对应 ROS setup.bash')
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation

from handeye_web.core import default_config
from handeye_web.sources import RosSource
from tests.synthetic_source import DemoSource


def test_image_info_and_historical_tf_over_real_ros():
    cfg = default_config(profile='handeye_synthetic_test')
    cfg['pose_source'] = 'tf'
    demo = DemoSource(cfg)
    # Render without annotations for detector input.
    demo.next()
    # A drawing from the underlying board is sufficient here; real PnP regression
    # uses perspective-projected boards in the main test suite.
    board = demo.board_image
    image = cv2.cvtColor(np.pad(board, ((100,100),(100,100)), constant_values=255), cv2.COLOR_GRAY2BGR)
    source = RosSource(cfg)
    pubnode = Node('handeye_synthetic_camera_test')
    image_pub = pubnode.create_publisher(Image, cfg['color_topic'], qos_profile_sensor_data)
    info_pub = pubnode.create_publisher(CameraInfo, cfg['camera_info_topic'], qos_profile_sensor_data)
    broadcaster = TransformBroadcaster(pubnode)
    bridge = CvBridge()
    output = None
    status = ''
    try:
        deadline = time.monotonic()+8
        while time.monotonic() < deadline:
            now = pubnode.get_clock().now().to_msg()
            message = bridge.cv2_to_imgmsg(image, encoding='bgr8')
            message.header.stamp = now
            message.header.frame_id = 'synthetic_color_optical_frame'
            info = CameraInfo()
            info.header = message.header
            info.height, info.width = image.shape[:2]
            info.k = [1200.,0.,info.width/2,0.,1200.,info.height/2,0.,0.,1.]
            info.d = [0.]*5
            info.distortion_model = 'plumb_bob'
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = cfg['base_frame']
            tf.child_frame_id = cfg['tool_frame']
            tf.transform.rotation.w = 1.
            tf.transform.translation.x = .42
            tf.transform.translation.z = .56
            broadcaster.sendTransform(tf)
            image_pub.publish(message)
            info_pub.publish(info)
            time.sleep(.03)
            try:
                canvas, output, status = source.next()
            except ValueError as error:
                status = str(error)
                continue
            if output:
                break
        assert output is not None, status
        assert output['camera']['frame_id'] == 'synthetic_color_optical_frame'
        assert np.allclose(np.array(output['T_base_tool'])[:3,3],[.42,0.,.56])
        assert output['reprojection_rmse_px'] < 3
        # Stop publication: the source must reject the frozen image after 1s.
        time.sleep(1.1)
        with pytest.raises(ValueError, match='陈旧'):
            source.next()
    finally:
        source.close()
        pubnode.destroy_node()
