# Copyright (C) 2021, Raffaello Bonghi <raffaello@rnext.it>
# All rights reserved
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright 
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its 
#    contributors may be used to endorse or promote products derived 
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nanosaur_msgs.msg import Eyes
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray


class Controller(Node):

    def __init__(self):
        super().__init__('nanosaur_ei_controller')
        # Node started
        self.get_logger().info("Hello nanosaur_ei_controller!")
        # Get frame size to follow
        self.declare_parameter("frame.width", 320.0)
        self.frame_width = self.get_parameter("frame.width").value
        self.declare_parameter("frame.height", 320.0)
        self.frame_height = self.get_parameter("frame.height").value
        # Gain eyes message
        self.declare_parameter("gain.eyes.x", 1)
        self.gain_eyes_x = self.get_parameter("gain.eyes.x").value
        self.declare_parameter("gain.eyes.y", 1)
        self.gain_eyes_y = self.get_parameter("gain.eyes.y").value
        #Init QoS
        qos_profile = QoSProfile(depth=5)
        # Create command Twist publisher
        self.pub_nav_ = self.create_publisher(Twist, '/nanosaur/nav_vel', qos_profile)
        # Create command eyes publisher
        self.pub_eyes_ = self.create_publisher(Eyes, '/nanosaur/eyes', qos_profile)
        # Subscribe to AprilTag Detection message
        self.subscription = self.create_subscription(
            Detection2DArray,
            'results',
            self.april_tag,
            1)

    def follower_stop(self):
        self.pub_nav_.publish(Twist())
        self.pub_eyes_.publish(Eyes())

    def move_eyes(self, error_x, error_y):
        eyes_msg = Eyes()
        # Convert center to eye movement
        eyes_msg.position.x = self.gain_eyes_x * error_x
        eyes_msg.position.y = self.gain_eyes_y * error_y
        # self.get_logger().info(f"[{eyes_msg.x:.0f}, {eyes_msg.y:.0f}]")
        # Wrap to Eyes message
        self.pub_eyes_.publish(eyes_msg)

    def detect_person(self, detections):
        person = {"detect": False, "px": 0., "py": 0., "size": 0.}
        for detection in detections:
            results = detection.results
            bbox = detection.bbox
            center = bbox.center
            for result in results:
                if result.id == "person":
                    person["detect"] = True
                    score = result.score
                    person["px"] = center.x #-(center.x - self.frame_width / 2.0)
                    person["py"] = center.y # (center.y - self.frame_height / 2.0)
                    # Size tag
                    # person["size"] = self.size_tag(detect.corners)
                    self.get_logger().info(f"{bbox}")
                    break
        return person

    def april_tag(self, msg):
        person = self.detect_person(msg.detections)
        self.get_logger().info(f"{person}")
        px = person["px"]
        py = person["py"]
        # Update eyes
        self.move_eyes(px, py)
        return


def main(args=None):
    rclpy.init(args=args)
    # Start Nanosaur
    controller = Controller()
    try:
        rclpy.spin(controller)
    except (KeyboardInterrupt, SystemExit):
        pass
    controller.follower_stop()
    # Destroy the node explicitly
    controller.destroy_node()
    rclpy.shutdown()
    print("EI controller node quit")


if __name__ == '__main__':
    main()
# EOF
