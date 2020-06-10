#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from turtlesim.msg  import Pose
from sensor_msgs.msg import LaserScan

class point:
	def __init__(self,x,y):
		self.x=x
		self.y=y

destination = point(-1, -1)
r = 100
K_lin=1 # wspolczynnik do wyznaczania predkosci liniowej
precision=0.1
travel_mode = 0 # 0 - wzdluz linii m; 1 - wzdluz sciany

# funkcja wywolywana przy przyjsciu danych ze skanera laserowego
def scan_callback(scan):
	print scan

# wyznaczanie docelowej wartosci theta oraz odleglosci robota od punktu docelowego w danym momencie
# T-aktualne polozenie zolwia, P-punkt docelowy
def calculate_needed_theta(T,P):
	global r
	delta_x=P.x-T.x
	delta_y=P.y-T.y
	r=math.sqrt(delta_x*delta_x+delta_y*delta_y)
	if r<precision: # jesli robot znajduje sie juz w punkcie docelowym, to nie trzeba wyznaczac wartosci theta
		return 10
	alfa=math.asin(delta_y/r)
	if P.x<T.x:
		if alfa>0:
			alfa=math.pi-alfa
		else:
			alfa=-math.pi-alfa
	return alfa

# funkcja wywolywana przy przyjsciu danych o lokalizacji robota
def odom_callback(odom):
	global new_vel
	pose = Pose()
	pose.x = odom.pose.pose.position.x
	pose.y = odom.pose.pose.position.y
	pose.theta = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w])[2]
	print "Pozycja x: ",odom.pose.pose.position.x
	print "Pozycja y: ",odom.pose.pose.position.y
	print "Pozycja theta: ",pose.theta

	needed_theta=calculate_needed_theta(point(pose.x,pose.y), destination)
	if r<precision: # robot dotarl do punktu docelowego
		print('Robot dotarl do celu')
		rospy.loginfo("Pozycja x: %8.2f",pose.x)
		rospy.loginfo("Pozycja y: %8.2f",pose.y)
		rospy.loginfo("Pozycja theta: %8.2f",pose.theta)
		new_vel.linear.x = 0.0
		new_vel.angular.z = 0.0
		return
	if travel_mode == 0:
		if abs(pose.theta-needed_theta) > precision:
			# obrot
			new_vel.linear.x = 0.0
			if needed_theta>(pose.theta-math.pi) and needed_theta<pose.theta:
				new_vel.angular.z=0.5
			else:
				new_vel.angular.z=-0.5
		else:
			# ruch do przodu
			new_vel.linear.x = K_lin*r
			new_vel.angular.z=0.0
	else:
		# dopisac cos tam travel_mode = 1

if __name__== "__main__":
	global new_vel
	new_vel = Twist()
	rospy.init_node('wr_zad', anonymous=True)
	print("ready")
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	rospy.Subscriber( '/odom' , Odometry, odom_callback)
	rospy.Subscriber( '/scan' , LaserScan, scan_callback)
	
	rate=rospy.Rate(10) # 10Hz
	while not rospy.is_shutdown():
		pub.publish(new_vel)#wyslanie predkosci zadanej
		rate.sleep()

	print("END")


