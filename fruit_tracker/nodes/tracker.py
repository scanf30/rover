#!/usr/bin/env python

PACKAGE = 'fruit_tracker'
import roslib;roslib.load_manifest(PACKAGE)
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge, CvBridgeError
import dynamic_reconfigure.client

lr1 = [0, 0, 0]
ur1 = [0, 0, 0]
lr2 = [0, 0, 0]
ur2 = [0, 0, 0]


def callback(config):

	global lr1
	global ur1
	global lr2
	global ur2

	hL1 = config['h_lower1']
	sL1 = config['s_lower1']
	vL1 = config['v_lower1']

	hL2 = config['h_lower2']
	sL2 = config['s_lower2']
	vL2 = config['v_lower2']

	hU1 = config['h_upper1']
	sU1 = config['s_upper1']
	vU1 = config['v_upper1']

	hU2 = config['h_upper2']
	sU2 = config['s_upper2']
	vU2 = config['v_upper2']

	lr1 = [int(hL1), int(sL1), int(vL1)]
	ur1 = [int(hU1), int(sU1), int(vU1)]

	lr2 = [int(hL2), int(sL2), int(vL2)]
	ur2 = [int(hU2), int(sU2), int(vU2)]

def publisher():

	global lr1
	global ur1
	global lr2
	global ur2

	rospy.init_node("fruit_tracker_node")
	rospy.wait_for_service("/fruit_tracker_server/set_parameters")
	
	client = dynamic_reconfigure.client.Client("fruit_tracker_server", timeout=30, config_callback=callback)


	imgPub = rospy.Publisher('fruitImg', Image, queue_size = 1)
	posPub = rospy.Publisher('posiTomate', Vector3, queue_size = 1)

	bridge = CvBridge()

	cap = cv2.VideoCapture(1)
	cap.set(3, 660)
	cap.set(4, 480)

	msg = Vector3()

	rate = rospy.Rate(60)

	while not rospy.is_shutdown():
		
		msg.x = 0
		msg.y = 0
		msg.z = 0		

		ret, frame = cap.read()
		blurred = cv2.GaussianBlur(frame, (11,11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

		lower_red1 = np.array(lr1)
		upper_red1 = np.array(ur1)
		lower_red2 = np.array(lr2)
		upper_red2 = np.array(ur2)

		mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
		mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

		mask = mask1 | mask2

		mask = cv2.erode(mask, None, iterations = 2)
		mask = cv2.dilate(mask, None, iterations = 2)

		(_, cnts, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

		cv2.drawContours(frame, cnts, -1, (0, 255, 0), 3)
		cv2.line(frame, (0, 160), (660, 160), (255, 0, 0), 3)
		cv2.line(frame, (0, 320), (660, 320), (255, 0, 0), 3)
		cv2.line(frame, (220, 0), (220, 480), (255, 0, 0), 3)
		cv2.line(frame, (440, 0), (440, 480), (255, 0, 0), 3)

		if len(cnts) > 0:
		
			c = max(cnts, key = cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			
			if (radius > 15):
				#cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
				#cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)

				offset = 20
			
				cv2.rectangle(frame,(int(x - radius - offset), int(y - radius - offset)),(int(x + radius + offset), int(y + radius + offset)),(0,255,0),3)
				cv2.rectangle(frame,(int(x - radius - offset - 2), int(y - radius - 15 - offset)),(int(x + 70), int(y - radius - offset)),(0,255,0),-1)

				#El mouse rojo tiene un diametro de 100 px a 0.56m de distancia.
				diameter = radius * 2
				dist = round(56 / (diameter), 2)

				font = cv2.FONT_HERSHEY_PLAIN
				text = 'Tomate! ' + str(dist) + 'm'
    				cv2.putText(frame, text, (int(x - radius - offset), int(y - radius - offset)), font, 1,(255,255,255),2,cv2.LINE_AA)

				

				if (diameter < 100):
					msg.x = 1
			
				if (x < 220):
					msg.y = -1
				elif (y > 440):
					msg.y = 1

				if (y < 160):
					msg.z = 1
				elif (y > 360):
					msg.z = -1

		posPub.publish(msg)

		
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


