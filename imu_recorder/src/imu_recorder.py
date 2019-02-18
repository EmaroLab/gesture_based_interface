#!/usr/bin/python

import rospy
import os
from sensor_msgs.msg import Imu
import numpy as np
import time
import json


class imuRecorder(object):

    def __init__(self, file_name, file_path):
        self.file_name = file_name
        self.file_path = file_path
        self.start_time = time.time()
        self.data = {"av_x":[], "av_y":[], "av_z":[], "la_x":[], "la_y":[], "la_z":[]}        
        self.last_acc_time = -1
        self.threshold = rospy.Duration.from_sec(0.15)

    def set_file_name(self, name, path):
        self.file_name = name
        if(path != ""):
            self.file_path = path
        else:
            self.file_path =  os.path.dirname(os.path.abspath(__file__))+"/"

    def save_file(self):
        if (len(recorder.data) != 0):
            print "saving in " + self.file_path
            fp_name = self.file_path + self.file_name  + ".json"

            exists = os.path.isfile(fp_name)
            if exists: 
                print "ATTENTION: the file already existed, name got changed"
                self.file_name = self.file_name + "_" + str(int(time.time()))
                fp_name = self.file_path + self.file_name  + ".json"

            with open(fp_name, 'w') as outfile:
                json_str = json.dumps(self.data, indent=4)
                outfile.write(json_str)
                print "\n" + self.file_name + ".json saved\n"         
        else:
            print "No recording to save"   

    def add_line(self, message):
        self.check_time(message)
        self.to_dict(message)
    
    def check_time(self, message):
        if (self.last_acc_time != -1):
            diff_acc = message.header.stamp - self.last_acc_time
            if (diff_acc > self.threshold):
                print self.file_name + "-" + "ATTENTION: delay" + str(diff_acc.to_nsec()) + " ns"

        self.last_acc_time = message.header.stamp

    def to_dict(self, msg):
        self.data['av_x'].append(msg.angular_velocity.x)
        self.data['av_y'].append(msg.angular_velocity.y)
        self.data['av_z'].append(msg.angular_velocity.z)
        self.data['la_x'].append(msg.linear_acceleration.x)
        self.data['la_y'].append(msg.linear_acceleration.y)
        self.data['la_z'].append(msg.linear_acceleration.z)


recorder = imuRecorder("new","./")
sub = None
count = 0

def callback(data):
    global recorder, count, sub
    recorder.add_line(data)
    print("\t%d.%d" % (data.header.stamp.secs, data.header.stamp.nsecs))
    count+=1
    if count == 40:
      sub.unregister()
      recorder.save_file()
      rospy.shutdown()

def main():
    rospy.init_node('imu_recorder', disable_signals=True)

    file_name = rospy.get_param('~file_name','new')
    file_path = rospy.get_param('~file_path','./')

    recorder.set_file_name(file_name, file_path)
  
    global sub
    sub = rospy.Subscriber("/imu_data", Imu, callback, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

