#!/usr/bin/env python

PACKAGE = 'fruit_tracker'
import roslib;roslib.load_manifest(PACKAGE)
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge, CvBridgeError
import dynamic_reconfigure.client

lr = [0, 0, 0]
ur = [0, 0, 0]


def callback(config):

	global lr
	global ur

	hL = config['h_lower']
	sL = config['s_lower']
	vL = config['v_lower']

	hU = config['h_upper']
	sU = config['s_upper']
	vU = config['v_upper']

	

	lr = [int(hL), int(sL), int(vL)]
	ur = [int(hU), int(sU), int(vU)]

def publisher():

	global lr
	global ur

	rospy.init_node("fruit_tracker_node")
	rospy.wait_for_service("/fruit_tracker_server/set_parameters")
	
	client = dynamic_reconfigure.client.Client("fruit_tracker_server", timeout=30, config_callback=callback)


	imgPub = rospy.Publisher('fruitImg', Image, queue_size = 1)
	yPub = rospy.Publisher('yAxis', Int16, queue_size = 1)

	bridge = CvBridge()

	lower_red = np.array(lr)
	upper_red = np.array(ur)

	cap = cv2.VideoCapture(1)
	cap.set(3, 660)
	cap.set(4, 480)

	msg = Int16()

	rate = rospy.Rate(60)

	while not rospy.is_shutdown():
		
		msg.data = 0		

		ret, frame = cap.read()
		blurred = cv2.GaussianBlur(frame, (11,11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

		lower_red = np.array(lr)
		upper_red = np.array(ur)

		mask = cv2.inRange(hsv, lower_red, upper_red)
		mask = cv2.erode(mask, None, iterations = 2)
		mask = cv2.dilate(mask, None, iterations = 2)

		(_, cnts, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

		#cv2.drawContours(frame, cnts, -1, (0, 255, 0), 3)
		cv2.line(frame, (0, 160), (660, 160), (255, 0, 0), 3)
		cv2.line(frame, (0, 320), (660, 320), (255, 0, 0), 3)

		if len(cnts) > 0:
		
			c = max(cnts, key = cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			
			if (radius > 10):
				#cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
				#cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)

				offset = 20
			
				cv2.rectangle(frame,(int(x - radius - offset), int(y - radius - offset)),(int(x + radius + offset), int(y + radius + offset)),(0,255,0),3)
				cv2.rectangle(frame,(int(x - radius - offset - 2), int(y - radius - 15 - offset)),(int(x + 70), int(y - radius - offset)),(0,255,0),-1)

				#El mouse rojo tiene un diametro de 100 px a 0.56m de distancia.
				dist = round(56 / (radius * 2), 2)

				font = cv2.FONT_HERSHEY_PLAIN
				text = 'Tomate! ' + str(dist) + 'm'
    				cv2.putText(frame, text, (int(x - radius - offset), int(y - radius - offset)), font, 1,(255,255,255),2,cv2.LINE_AA)
			
				if (y < 160):
					msg.data = 1
				elif (y > 360):
					msg.data = -1

		yPub.publish(msg)

		
		try:
			imgPub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
		except CvBridgeError as e:
			print(e)

		rate.sleep()

	cap.release()
	cv2.destroyAllWindows()


if __name__ == '__main__':
	
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass


