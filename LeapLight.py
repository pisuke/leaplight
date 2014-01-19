#!/usr/bin/env python
# Control DMX luminaire with ENTTEC USB PRO and Leap Motion using gestures
# 2014-01-18 Francesco Anselmo

import sys, serial, struct, numpy, time, math, random
import Leap, sys
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture

# DMX stuff
class DMXDevice(object):
	def __init__(self, start, length):
		self.start, self.length = start, length
		self.values = [0] * self.length

	def set(self, chan, value):
		"""set the value of this channel to value (relative channel number)"""
		self.values[chan] = value

	def pack(self, buf):
		"""modify the passed buffer in place"""
		for index in range(self.length):
			buf[self.start+index] = self.values[index]

	def __str__(self):
		return "<DMXDevice start=%d, length=%d>" % (self.start, self.length)

class DMXManager(object):
	def __init__(self, port):
		self.s = serial.Serial(port)
		self.buf = numpy.zeros((128,), dtype='B')
		self.devices = []

	def append(self, device):
		self.devices.append(device)

	def send(self):
		for device in self.devices:
			device.pack(self.buf)
		msg = struct.pack("<BBH 128s B",
			0x7e, 6, 128, 
			self.buf.tostring(),
			0xe7
		)
		self.s.write(msg)


# Leap Motion stuff
class DMXLeapListener(Leap.Listener):
	#def __init__(self):
	
	def on_init(self, controller):
		self.port = sys.argv[1]
		self.manager = DMXManager(self.port)
		self.light_0 = DMXDevice(start=1, length=2)
		self.light_1 = DMXDevice(start=2, length=2)
		self.manager.append(self.light_0)
		self.manager.append(self.light_1)

		print "Initialized"

	def on_connect(self, controller):
		print "Connected"

		# Enable gestures
		controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE);
		controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP);
		controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP);
		controller.enable_gesture(Leap.Gesture.TYPE_SWIPE);

	def on_disconnect(self, controller):
		# Note: not dispatched when running in a debugger.
		print "Disconnected"

	def on_exit(self, controller):
		print "Exited"

	def on_frame(self, controller):
		# Get the most recent frame and report some basic information
		frame = controller.frame()

		#print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % (
		#	  frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools), len(frame.gestures()))

		if not frame.hands.is_empty:
			# Get the first hand
			hand = frame.hands[0]

			# Check if the hand has any fingers
			fingers = hand.fingers
			if not fingers.is_empty:
				# Calculate the hand's average finger tip position
				avg_pos = Leap.Vector()
				for finger in fingers:
					avg_pos += finger.tip_position
				avg_pos /= len(fingers)
				print "Hand has %d fingers, average finger tip position: %s" % (
					  len(fingers), avg_pos)
				x = avg_pos[0]
				x1 = abs(100-int(x))
				x2 = abs(100+int(x))
				self.light_0.set(0, x1)
				self.light_1.set(0, x2)
				self.manager.send()
				time.sleep(.01)


			# Get the hand's sphere radius and palm position
			print "Hand sphere radius: %f mm, palm position: %s" % (
				  hand.sphere_radius, hand.palm_position)

			# Get the hand's normal vector and direction
			normal = hand.palm_normal
			direction = hand.direction

			# Calculate the hand's pitch, roll, and yaw angles
			print "Hand pitch: %f degrees, roll: %f degrees, yaw: %f degrees" % (
				direction.pitch * Leap.RAD_TO_DEG,
				normal.roll * Leap.RAD_TO_DEG,
				direction.yaw * Leap.RAD_TO_DEG)

			# Gestures
			for gesture in frame.gestures():
				if gesture.type == Leap.Gesture.TYPE_CIRCLE:
					circle = CircleGesture(gesture)

					# Determine clock direction using the angle between the pointable and the circle normal
					if circle.pointable.direction.angle_to(circle.normal) <= Leap.PI/4:
						clockwiseness = "clockwise"
					else:
						clockwiseness = "counterclockwise"

					# Calculate the angle swept since the last frame
					swept_angle = 0
					if circle.state != Leap.Gesture.STATE_START:
						previous_update = CircleGesture(controller.frame(1).gesture(circle.id))
						swept_angle =  (circle.progress - previous_update.progress) * 2 * Leap.PI

					print "Circle id: %d, %s, progress: %f, radius: %f, angle: %f degrees, %s" % (
							gesture.id, self.state_string(gesture.state),
							circle.progress, circle.radius, swept_angle * Leap.RAD_TO_DEG, clockwiseness)

				if gesture.type == Leap.Gesture.TYPE_SWIPE:
					swipe = SwipeGesture(gesture)
					print "Swipe id: %d, state: %s, position: %s, direction: %s, speed: %f" % (
							gesture.id, self.state_string(gesture.state),
							swipe.position, swipe.direction, swipe.speed)

				if gesture.type == Leap.Gesture.TYPE_KEY_TAP:
					keytap = KeyTapGesture(gesture)
					print "Key Tap id: %d, %s, position: %s, direction: %s" % (
							gesture.id, self.state_string(gesture.state),
							keytap.position, keytap.direction )

				if gesture.type == Leap.Gesture.TYPE_SCREEN_TAP:
					screentap = ScreenTapGesture(gesture)
					print "Screen Tap id: %d, %s, position: %s, direction: %s" % (
							gesture.id, self.state_string(gesture.state),
							screentap.position, screentap.direction )

		if not (frame.hands.is_empty and frame.gestures().is_empty):
			print ""

	def state_string(self, state):
		if state == Leap.Gesture.STATE_START:
			return "STATE_START"

		if state == Leap.Gesture.STATE_UPDATE:
			return "STATE_UPDATE"

		if state == Leap.Gesture.STATE_STOP:
			return "STATE_STOP"

		if state == Leap.Gesture.STATE_INVALID:
			return "STATE_INVALID"


def main():

	# Create a sample listener and controller
	listener = DMXLeapListener()
	controller = Leap.Controller()

	# Have the sample listener receive events from the controller
	controller.add_listener(listener)

	# Keep this process running until Enter is pressed
	print "Press Enter to quit..."
	sys.stdin.readline()

	# Remove the sample listener when done
	controller.remove_listener(listener)


if __name__ == "__main__":
	main()
	
	
"""	
  while True:
	intensity = 128*math.sin(time.time())+128
	light_0.set(0, int(intensity))
	light_1.set(0, int(intensity))
	#for light in light_0, light_1:
	#  for color in range(3):
	#	light.set(color, random.randintil.com(0, 255))
	manager.send()
	time.sleep(.01)
"""
