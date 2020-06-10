#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from turtlesim.msg  import Pose
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
import tf2_ros
import tf

global new_path
global tfBuffer
global listener
global robot_pose
global pose
def get_pose():
	global robot_pose
	global pose
	got_transform = False
	# pozycja robota -- transformacja z ukladu mapy do ukladu robota
	while not got_transform:
		try:
		#rospy.sleep(4)
			trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time(), rospy.Duration(4.0))
			got_transform = True
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			rate.sleep()
	
	euler = tf.transformations.euler_from_quaternion([trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w])
	theta = euler[2]
	robot_pose = [trans.transform.translation.x, trans.transform.translation.y, theta]
	pose = Pose()
	pose.x = trans.transform.translation.x
	pose.y = trans.transform.translation.y
	pose.theta = theta
	print trans.transform
	return trans.transform

def get_path(x,y):
	global new_path
	start =PoseStamped()
	goal =PoseStamped()
	start.header.stamp = rospy.Time.now()
	start.header.frame_id="map"
	# pozycja robota -- transformacja z ukladu mapy do ukladu robota
	trans = get_pose()
	# pozycja robota jako poczatek sciezki
	start.pose.position.x = trans.translation.x
	start.pose.position.y = trans.translation.y
	start.pose.orientation = trans.rotation
	# cel ruchu robota pobrany z wiadomosci 
	goal.header.frame_id="map"
	goal.header.stamp = rospy.Time.now()
	goal.pose.position.x=x
	goal.pose.position.y = y
	# czekamy az usluga planowania bedzie dostepna
	rospy.wait_for_service('/move_base/GlobalPlanner/make_plan' )
	try :
		# Tworzymy obiekt klienta uslugi
		get_plan = rospy.ServiceProxy('/move_base/GlobalPlanner/make_plan',
						GetPlan)
		# wolamy usluge okreslajac zadanie
		resp=get_plan(start, goal, 1)
		print resp
		# zapisujemy sciezke do zmiennej globalnej
		new_path = resp.plan
		#print new_path
	except rospy.ServiceException , e:
		print"Service call failed: %s "%e


if __name__== "__main__":
	global new_path
	global tfBuffer
	global listener
	global pose
	new_path = Path()
	new_vel = Twist()
	rospy.init_node('zad3', anonymous=True)
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	print("ready")
	rate=rospy.Rate(10) # 10Hz
	pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
	get_path(2,2)
	while not rospy.is_shutdown():
	# Wstawic kod odpowiedzialny za aktualizację zmiennej przechowujacej pozycje
	# ...   
		if not len(new_path.poses) == 0:
			# dostep do pozycji robota jest poprzez zmienna globalna 'pose'
			#
			#
			#
			# petla wyznaczajaca kolejne predkosci zadane dla robota 
			# na podstawie sciezki, ktorej elementy odczytuje sie
			# poprzez: 
			# 	new_path.poses[i].pose.position.x = wspolrzedna 'x' punktu
			#	new_path.poses[i].pose.position.y = wspolrzedna 'y' punktu
			# gdzie 'i' jest identyfikatorem punktu sciezki. 
			# Dlugosc sciezki odczytuje sie poprzez: len(new_path.poses)
			# ...
			# ... 
			# ...
			# wyslanie nowej predkosci zadanej
			pub.publish(new_vel)#wyslaniepredkoscizadanej

		rate.sleep()

	print("END")

