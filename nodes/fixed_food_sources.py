#!/usr/bin/env python
from __future__ import print_function
import Queue
import threading
import rospy
import std_msgs.msg
import numpy as np

from food_region import FoodRegion
from em3242_angle_sensor_ros.msg import EM3242_AngleSensorData


class FixedFoodSourcesNode(object):

    DEFAULT_WHEEL_RADIUS = 5.0
    DEFAULT_FOOD_PARAMS = { 
            'position_list': [-100, 100], 
            'width': 10.0, 
            'led': {
                'pin': 3, 
                'on_value': 20,
                'off_value': 0, 
                'on_duration': 1.0,
                'refractory_duration': 1.0,
                }, 
            }

    def __init__(self):

        rospy.init_node('fixed_food_source')

        self.lock = threading.Lock()
        self.angle_data_queue = Queue.Queue() 
        self.start_time = rospy.Time.now().to_time()

        self.wheel_radius = rospy.get_param('/flywheel/wheel_radius', self.DEFAULT_WHEEL_RADIUS)
        self.food_param = rospy.get_param('/flywheel/food', self.DEFAULT_FOOD_PARAMS) 
        self.create_food_regions()
        rospy.on_shutdown(self.clean_up)

        self.angle_data_sub = rospy.Subscriber('/em3242_angle_sensor_data', EM3242_AngleSensorData, self.on_angle_data) 


    def clean_up(self):
        for food_region in self.food_region_list:
            food_region.led_scheduler.turn_off_led()


    def create_food_regions(self):
        self.food_region_list = []
        for index, position in enumerate(self.food_param['position_list']):
            food_region_param = {
                    'index': index,
                    'position': position, 
                    'width': self.food_param['width'], 
                    'led': self.food_param['led']
                    }
            self.food_region_list.append(FoodRegion(food_region_param))

    def on_angle_data(self,data):
        self.angle_data_queue.put(data)

    def run(self):
        while not rospy.is_shutdown():

            # Get current fly position
            angle_data = self.angle_data_queue.get()
            fly_position = np.deg2rad(angle_data.cumulative_angle)*self.wheel_radius

            ros_time_now = rospy.Time.now()
            current_time = ros_time_now.to_time()
            elapsed_time = current_time - self.start_time 
            print('t = {:1.2f}, pos = {:1.2f}'.format(elapsed_time, fly_position))

            for food_region in self.food_region_list:
                food_region.update(elapsed_time,fly_position)



# ---------------------------------------------------------------------------------------
if __name__ == '__main__':

    node = FixedFoodSourcesNode()
    node.run()

