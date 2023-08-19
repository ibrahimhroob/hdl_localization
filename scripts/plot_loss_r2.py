#!/usr/bin/python

import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Float32

MAX_BUFFER_SIZE = 50 

class Plotter(object):
	def __init__(self):
		plt.ion()
		plt.show(block=False)

		self.loss_buffer = []
		self.r2_buffer = []
		self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
		
		self.loss_sub = rospy.Subscriber('/model_loss', Float32, self.loss_callback)
		self.r2_sub = rospy.Subscriber('/model_r2', Float32, self.r2_callback)

	def loss_callback(self, data):
		print(data)
		self.loss_buffer.append(data.data)
		if len(self.loss_buffer) > MAX_BUFFER_SIZE:
			self.loss_buffer = self.loss_buffer[-MAX_BUFFER_SIZE:]

		
	def r2_callback(self, data):
		print(data)
		self.r2_buffer.append(data.data)
		if len(self.r2_buffer) > MAX_BUFFER_SIZE:
			self.r2_buffer = self.r2_buffer[-MAX_BUFFER_SIZE:]


	def timer_callback(self, event):
		if len(self.r2_buffer) < 2:
			return

		plt.clf()
		# plt.figure(figsize=(10, 6))
		plt.subplot(2, 1, 1)
		plt.plot(self.loss_buffer, '-b', label='Loss')
		plt.xlabel('Time')
		plt.ylabel('Loss')
		plt.title('Model Loss')
		plt.legend()

		plt.subplot(2, 1, 2)
		plt.plot(self.r2_buffer, '-g', label='R2 Score')
		plt.xlabel('Time')
		plt.ylabel('R2 Score')
		plt.title('Model R2 Score')
		plt.legend()

		plt.tight_layout()
		# plt.show()
	

		# plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), ncol=len(errors))
		plt.gcf().canvas.flush_events()
		# pyplot.pause(0.0001)


def main():
	# rospy.init_node('status_plotter')
	rospy.init_node('loss_r2_plotting_node', anonymous=True)
	node = Plotter()
	rospy.spin()

if __name__ == '__main__':
	main()
