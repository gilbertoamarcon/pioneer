#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

np.warnings.filterwarnings('ignore')

def sonarPC2Callback(msg, args):
	(pub, split_distance_threshod, hits, robot_radius, max_range) = args
	newscan = msg
	newscan.header.stamp = rospy.get_rostime()
	newscan.header.frame_id = 'rp_laser'
	newscan.ranges = enhance_ranges(msg.ranges, split_distance_threshod, hits, robot_radius, max_range)
	pub.publish(newscan)

def enhance_ranges(ranges, split_distance_threshod, hits, robot_radius, max_range):
	xy = get_xy(ranges)
	sxy = get_split(xy, split_distance_threshod=split_distance_threshod)
	robots = get_robots(sxy, hits=hits)
	return get_new_ranges(ranges, robots, robot_radius=robot_radius, max_range=max_range)

def get_xy(ranges):
	t = np.linspace(0.0, 2*np.pi, len(ranges), endpoint=False)
	x = ranges * np.cos(t)
	y = ranges * np.sin(t)
	return np.transpose(np.concatenate(((x,),(y,))))

def get_split(xy, split_distance_threshod):
	prior = xy[0,:]
	split = []
	for i in range(1,len(xy)):
		current = xy[i,:]
		if not np.isinf(current).any() and not np.isnan(current).any():
			if np.linalg.norm(current-prior) > split_distance_threshod:
				split.append(i)
			prior = current
	return np.split(xy, split)

def get_robots(sxy, hits):
	robots = []
	ctr = 0
	for s in sxy:
		size = len(s)
		if size < hits:
			robots.append((ctr,ctr+size))
		ctr += size
	return robots

def get_new_ranges(ranges, robots, robot_radius, max_range):
	new_ranges = np.array(ranges)
	for r in robots:
		rng = get_range(r, ranges)
		if rng < max_range:
			mean_idx = get_mean_idx(r, ranges)
			pad = 1+int(180.0*(robot_radius/rng)/np.pi)
			for i in range(mean_idx-pad,mean_idx+pad):
				alpha = 1.0-float(np.abs(i-mean_idx))/pad
				displacement = robot_radius*(alpha**0.25)
				if i>=0 and i<len(ranges):
					new_ranges[i] = rng-displacement
	return new_ranges

def get_range(r, ranges):
	raw = ranges[r[0]:r[1]]
	clean = np.array([
		r
		for r in raw
		if not np.isinf(r) and not np.isnan(r)
	])
	return np.mean(clean)

def get_mean_idx(r, ranges):
	clean = np.array([
		i
		for i in range(r[0],r[1])
		if not np.isinf(ranges[i]) and not np.isnan(ranges[i])
	])
	return int(np.mean(clean))

def main():
	rospy.init_node('laser_publisher')
	split_distance_threshod	= rospy.get_param('~split_distance_threshod')
	hits					= rospy.get_param('~hits')
	robot_radius			= rospy.get_param('~robot_radius')
	max_range				= rospy.get_param('~max_range')
	pub = rospy.Publisher('/base_scan', LaserScan, queue_size=10, latch=True)
	rospy.Subscriber('/scan', LaserScan, sonarPC2Callback, (pub, split_distance_threshod, hits, robot_radius, max_range))
	rospy.spin()

if __name__ == '__main__':
	main()
