#!/usr/bin/env python

import sys, csv
import rosbag

bag_file = sys.argv[1]
result_topic = '/sr_pcl_tracker/result/points'
total_topic = '/sr_pcl_tracker/cloud_downsampled/points'

bag = rosbag.Bag(bag_file)

results = []

for topic, msg, t in bag.read_messages(topics=[result_topic, total_topic]):
    row = { 't': t }
    if topic == result_topic:
        row['result'] = len(msg.data) / len(msg.fields)
    elif topic == total_topic:
        row['total'] = len(msg.data) / len(msg.fields)
    results.append(row)

bag.close()

writer = csv.writer(sys.stdout, delimiter=',')
writer.writerow(['bag_time', 'total_points', 'result_points'])
for res in results:
    out_row = { 't': str(res['t'].secs) + "." + str(res['t'].nsecs) }
    out_row['total'] = res['total'] if 'total' in res else None
    out_row['result'] = res['result'] if 'result' in res else None
    writer.writerow([out_row['t'], out_row['total'], out_row['result']])
