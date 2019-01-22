# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy

import baxter_interface

from baxter_interface import CHECK_VERSION


class JointRecorder(object):
    def __init__(self, filename):
        """
        Records joint data to a file.
        """
        self._filename = filename
        self._start_time = rospy.get_time()
        self._done = True

        self._limb_left = baxter_interface.Limb("left")
        self._limb_right = baxter_interface.Limb("right")
        self._gripper_left = baxter_interface.Gripper("left", CHECK_VERSION)
        self._gripper_right = baxter_interface.Gripper("right", CHECK_VERSION)
        self._io_left_lower = baxter_interface.DigitalIO('left_lower_button')
        self._io_left_upper = baxter_interface.DigitalIO('left_upper_button')
        self._io_right_lower = baxter_interface.DigitalIO('right_lower_button')
        self._io_right_upper = baxter_interface.DigitalIO('right_upper_button')

        # Verify Grippers Have No Errors and are Calibrated
        if self._gripper_left.error():
            self._gripper_left.reset()
        if self._gripper_right.error():
            self._gripper_right.reset()
        if (not self._gripper_left.calibrated() and
            self._gripper_left.type() != 'custom'):
            self._gripper_left.calibrate()
        if (not self._gripper_right.calibrated() and
            self._gripper_right.type() != 'custom'):
            self._gripper_right.calibrate()
            
        self._joints_left = self._limb_left.joint_names()
        self._joints_right = self._limb_right.joint_names()
        self._f = open(self._filename, 'w')
        self._f.write('time,')
        self._f.write(','.join([j for j in self._joints_left]) + ',')
        self._f.write('left_gripper,')
        self._f.write(','.join([j for j in self._joints_right]) + ',')
        self._f.write('right_gripper\n')
        self._done = False

    def _time_stamp(self):
        return rospy.get_time() - self._start_time

    def stop(self):
        """
        Stop recording.
        """
        self._done = True
        self._f.close()

    def done(self):
        """
        Return whether or not recording is done.
        """
        if rospy.is_shutdown():
            self.stop()
        return self._done

    def record_instance(self):
        """
        Records the current joint positions to a csv file if outputFilename was
        provided at construction this function will record the latest set of
        joint angles in a csv format.

        This function does not test to see if a file exists and will overwrite
        existing files.
        """

        if self._filename:
            if not self.done():
                # Look for gripper button presses
                if self._io_left_lower.state:
                    self._gripper_left.open()
                elif self._io_left_upper.state:
                    self._gripper_left.close()
                if self._io_right_lower.state:
                    self._gripper_right.open()
                elif self._io_right_upper.state:
                    self._gripper_right.close()
                angles_left = [self._limb_left.joint_angle(j)
                               for j in self._joints_left]
                angles_right = [self._limb_right.joint_angle(j)
                                for j in self._joints_right]

                self._f.write("%f," % (self._time_stamp(),))

                self._f.write(','.join([str(x) for x in angles_left]) + ',')
                self._f.write(str(self._gripper_left.position()) + ',')

                self._f.write(','.join([str(x) for x in angles_right]) + ',')
                self._f.write(str(self._gripper_right.position()) + '\n')
