from __future__ import print_function
import sys
import rosbag
import matplotlib.pyplot as plt

bag = rosbag.Bag(sys.argv[1])

bag_info = bag.get_type_and_topic_info()
print()
print('topics')
print('------')
for topic in bag_info.topics:
    print('  {}'.format(topic))
print()

time_list = []
pos_list = []

for topic, msg, t in bag.read_messages(topics=['/flywheel_data']):
    print(msg)
    pos_list.append( msg.position)
    time_list.append(msg.elapsed_time)

plt.plot(time_list,pos_list)
plt.grid('on')
plt.xlabel('t (sec)')
plt.ylabel('position (mm)')
plt.show()



