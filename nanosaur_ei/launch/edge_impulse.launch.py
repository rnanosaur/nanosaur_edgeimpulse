# Copyright (C) 2022, Raffaello Bonghi <raffaello@rnext.it>
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


from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ei_node = Node(
        package='edgeimpulse_ros',
        executable='image_classification',
        name='image_classification',
        namespace='nanosaur',
        parameters=[{
            'model.filepath': '/opt/ei/models/ssd/person_detection/person-detection-linux-aarch64-v42.eim',
        }],
        remappings=[('/detection/input/image', 'camera/color/image_raw')],
    )
    
    ei_controller_node = Node(
        package='nanosaur_ei_controller',
        namespace='nanosaur',
        executable='nanosaur_ei_controller',
        name='nanosaur_ei_controller',
    )


    ld = LaunchDescription()

    ld.add_action(ei_node)
    ld.add_action(ei_controller_node)

    return ld
# EOF
