#!/usr/bin/python

import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Float32

MAX_BUFFER_SIZE = 2000 

class Plotter(object):
	def __init__(self):
		plt.ion()
		plt.show(block=False)

		self.scan_filter_buffer = []
		self.map_scan_filter_buffer = []
		self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

		self.sfr = rospy.Subscriber('/scan_filter_ratio', Float32, self.scan_callback)
		self.msr = rospy.Subscriber('/map_to_scan_ratio', Float32, self.map_callback)

	def scan_callback(self, data):
		print(data)
		self.scan_filter_buffer.append(data.data)
		if len(self.scan_filter_buffer) > MAX_BUFFER_SIZE:
			self.scan_filter_buffer = self.scan_filter_buffer[-MAX_BUFFER_SIZE:]

		
	def map_callback(self, data):
		print(data)
		self.map_scan_filter_buffer.append(data.data)
		if len(self.map_scan_filter_buffer) > MAX_BUFFER_SIZE:
			self.map_scan_filter_buffer = self.map_scan_filter_buffer[-MAX_BUFFER_SIZE:]


	def timer_callback(self, event):
		if len(self.map_scan_filter_buffer) < 2:
			return

		plt.clf()
		# plt.figure(figsize=(10, 6))
		plt.subplot(2, 1, 1)
		plt.plot(self.scan_filter_buffer, '-b', label='SFR')
		plt.xlabel('Time')
		plt.ylabel('Scan filter')
		plt.title('Scan dynamic points filter ration')
		plt.legend()

		plt.subplot(2, 1, 2)
		plt.plot(self.map_scan_filter_buffer, '-g', label='Map to scan coord')
		plt.xlabel('Time')
		plt.ylabel('Map to scan coord ratio')
		plt.title('Map to scan filter performance')
		plt.legend()

		plt.tight_layout()
		# plt.show()
	

		# plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), ncol=len(errors))
		plt.gcf().canvas.flush_events()
		# pyplot.pause(0.0001)


def main():
	# rospy.init_node('status_plotter')
	rospy.init_node('filter_ratio_plotting_node', anonymous=True)
	node = Plotter()
	rospy.spin()

if __name__ == '__main__':
	main()
