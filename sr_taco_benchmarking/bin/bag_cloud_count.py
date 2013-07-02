#!/usr/bin/env python

import sys, csv
import rosbag

bag_file = sys.argv[1]
result_topic = '/sr_pcl_tracker/result/points'
total_topic = '/sr_pcl_tracker/cloud_downsampled/points'

bag = rosbag.Bag(bag_file)

results = []

for topic, msg, t in bag.read_messages(topics=[result_topic, total_topic]):
    row = { 't': str(t.secs) + "." + str(t.nsecs), 'total': None, 'result': None  }
    if topic == result_topic:
        row['result'] = msg.height * msg.width
    elif topic == total_topic:
        row['total'] = msg.height * msg.width
    results.append(row)

bag.close()

writer = csv.writer(sys.stdout, delimiter=',')
writer.writerow(['bag_time', 'total_points', 'result_points'])
for row in results:
    writer.writerow([row['t'], row['total'], row['result']])
