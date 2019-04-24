#!/usr/bin/env python
from __future__ import print_function
import os
import Queue
import yaml
import rospy
import std_msgs.msg
import numpy as np

from food_region import FoodRegion
from em3242_angle_sensor_ros.msg import EM3242_AngleSensorData

from flywheel_ros.msg import FlyWheelData
from flywheel_ros.msg import FoodRegionData

class FixedFoodSourcesNode(object):


    Default_Param_File = 'fixed_food_params.yaml'
    

    def __init__(self):
        self.start_position = None 

        rospy.init_node('fixed_food_source')
        self.get_param()
        self.create_food_regions()
        self.start_time = rospy.Time.now().to_time()
        self.angle_data_queue = Queue.Queue() 
        self.flywheel_data_pub = rospy.Publisher('/flywheel_data', FlyWheelData, queue_size=10) 
        self.angle_data_sub = rospy.Subscriber('/em3242_angle_sensor_data', EM3242_AngleSensorData, self.on_angle_data) 

    def get_param(self):
        self.param = rospy.get_param('/flywheel', None)
        if self.param is None:
            param_file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),self.Default_Param_File)
            with open(param_file_path,'r') as f:
                self.param = yaml.load(f)
        print(self.param)
        print()

    def create_food_regions(self):
        self.food_region_list = []
        for index, position in enumerate(self.param['food_sources']['position_list']):
            food_region_param = {
                    'index': index,
                    'position': position, 
                    'width': self.param['food_sources']['width'], 
                    'led': self.param['food_sources']['led']
                    }
            self.food_region_list.append(FoodRegion(food_region_param))

    def on_angle_data(self,data):
        self.angle_data_queue.put(data)

    def run(self):

        while not rospy.is_shutdown():

            try:
                angle_data = self.angle_data_queue.get(block=False,timeout=1.0)
            except Queue.Empty:
                continue

            if self.start_position is None:
                self.start_position = np.deg2rad(angle_data.cumulative_angle)*self.param['wheel_radius']

            fly_position = np.deg2rad(angle_data.cumulative_angle)*self.param['wheel_radius']
            fly_position -= self.start_position

            # Get ROS time and elapsed time and update food regions
            ros_time_now = rospy.Time.now()
            current_time = ros_time_now.to_time()
            elapsed_time = current_time - self.start_time 
            for food_region in self.food_region_list:
                food_region.update(elapsed_time,fly_position)

            # Publish data for flywheel
            flywheel_data = FlyWheelData()
            flywheel_data.header.stamp = ros_time_now
            flywheel_data.position = fly_position
            flywheel_data.elapsed_time = elapsed_time
            for food_region in self.food_region_list:
                region_data = FoodRegionData()
                region_data.location = food_region.position
                region_data.width = food_region.width
                region_data.contains = food_region.contains(fly_position)
                region_data.led_data.on = food_region.led_scheduler.led_on
                region_data.led_data.value = food_region.led_scheduler.value
                region_data.led_data.pin = food_region.led_scheduler.pin
                flywheel_data.region_data_array.append(region_data)
            self.flywheel_data_pub.publish(flywheel_data)

            print('t = {:1.2f}, pos = {:1.2f}'.format(elapsed_time, fly_position))

# ---------------------------------------------------------------------------------------
if __name__ == '__main__':

    node = FixedFoodSourcesNode()
    node.run()

