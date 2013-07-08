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
writer.writerow(['bag_time', 'total_points', 'result_points_raw'])
# We track the last row to fill in gaps in the current row using the last value
# seen. ie the last value published
last_row = { 't': 0.0, 'total': None, 'result': None  }
for row in results:
    this_row = {}
    for k in row.keys():
        this_row[k] = row[k] if row[k] else last_row[k]
    writer.writerow([this_row['t'], this_row['total'], this_row['result']])
    last_row = this_row
