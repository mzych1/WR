#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from turtlesim.msg  import Pose
import math

class point:
	def __init__(self,x,y):
		self.x=x
		self.y=y

# lista pnktow do ktorych zolw ma dotrzec
points_list=[point(-1,-1), point(1,0), point(0,0), point(0,1), point(0.2, 8.75), point(0.2,0.2)] 
point_index=0 # indeks punktu docelowego
r=100 # odleglosc zolwia od punktu docelowego (poczatkowo ustawiam dowolna wartosc, potem zostanie ona wyznaczona)
K_lin=1 # wspolczynnik do wyznaczania predkosci liniowej
K_ang=0.5 # wspolczynnik do wyznaczania predkosci katowej
precision=0.1
new_vel = Twist()

# wyznaczanie docelowej wartosci theta oraz odleglosci zolwia od punktu docelowego w danym momencie
# T-aktualne polozenie zolwia, P-punkt docelowy
def calculate_needed_theta(T,P):
	global r
	delta_x=P.x-T.x
	delta_y=P.y-T.y
	r=math.sqrt(delta_x*delta_x+delta_y*delta_y)
	if r<precision: # jesli zolw znajduje sie juz w punkcie docelowym, to nie trzeba wyznaczac wartosci theta
		return 10
	alfa=math.asin(delta_y/r)
	if P.x<T.x:
		if alfa>0:
			alfa=math.pi-alfa
		else:
			alfa=-math.pi-alfa
	return alfa

def callback(odom):
	global new_vel
	global point_index
	new_vel = Twist()
    	pose = Pose()
	pose.x = odom.pose.pose.position.x
	pose.y = odom.pose.pose.position.y
	pose.theta = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w])[2]

	if point_index>=len(points_list): # zolw dotarl juz do wszystkich punktow z listy
		new_vel.linear.x = 0.0
		new_vel.angular.z = 0.0
		return

	needed_theta=calculate_needed_theta(point(pose.x,pose.y), points_list[point_index])
	if r<precision: # zolw dotarl do punktu docelowego
		print('\n-------------------------------------------------------------------')
		print("\nZolw dotarl do"),
		print(str(point_index+1)),
		print(" punktu z listy")
		rospy.loginfo("Pozycja x: %8.2f",pose.x)
		rospy.loginfo("Pozycja y: %8.2f",pose.y)
		rospy.loginfo("Pozycja theta: %8.2f",pose.theta)
		point_index=point_index+1 # indeks nowego punktu docelowego
		return

	print('punkt docelowy:		'+str(points_list[point_index].x)+'	'+str(points_list[point_index].y))
	print('polozenie:	' + str(pose.x)+'	'+str(pose.y))
	print('')
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

if __name__== "__main__":
	global new_vel
	new_vel = Twist()
	rospy.init_node('zad2', anonymous=True)
	print("ready")
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	rospy.Subscriber( '/odom' , Odometry, callback)
	
	rate=rospy.Rate(10) # 10Hz
	while not rospy.is_shutdown():
		pub.publish(new_vel) # wyslanie predkosci zadanej
		rate.sleep()

	print("END")
