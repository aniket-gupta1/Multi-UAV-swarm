import rospy
from geometry_msgs.msg import Twist, PoseStamped
import math 
import time
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from mavros_msgs.msg import GlobalPositionTarget
from math import radians, cos, sin, asin, sqrt 
import numpy as np
from pymavlink import mavutil
from mavros import setpoint as SP
import utm
from sensor_msgs.msg import NavSatFix

dt=1
T=500000

N=5
pos=[[38.146200,-76.428387],[38.145313,-76.429119],[38.149222,-76.429483],[38.150233,-76.430855],[38.150233,-76.430855]]


UAV_pos = np.zeros((3,1))


class Swarm_Bot:

	def __init__(self, ID):
		# Create a node with name 'UAV'+ ID and make sure it is a unique node
		#rospy.init_node('UAV'+ str(ID), anonymous=True)

		# Publisher to publish to the topic '/UAVID/mavros/setpoint_position' and '/UAVID/mavros/setpoint_position' 
		self.waypoint_publisher = rospy.Publisher('/UAV'+str(ID)+'/mavros/setpoint_position/local', PoseStamped, queue_size=10)
	
	def callback(data,data2):
		UAV_pos[0]=Float64(data2.latitude)
		UAV_pos[1]=Float64(data2.longitude)
		UAV_pos[2]=Float64(data2.altitude)

	def get_pos(self,ID):
		self.Position_subscriber = rospy.Subscriber('/UAV'+str(ID)+'/mavros/global_position/global', NavSatFix, self.callback)
		return UAV_pos

	def attitude_control(self, pos, v):
		pos_x = pos[0]
		pos_y = pos[1]
		pos_z = 30

		print "passing_waypoints"
		attitude = SP.PoseStamped()

		attitude.header.seq = 2
		attitude.header.stamp.secs = 2
		attitude.header.stamp.nsecs = 0
		attitude.header.frame_id = ''
		attitude.pose.position.x = pos_x
		attitude.pose.position.y = pos_y
		attitude.pose.position.z = pos_z
		attitude.pose.orientation.x = 0
		attitude.pose.orientation.y = 0
		attitude.pose.orientation.z = 0
		self.waypoint_publisher.publish(attitude)

	def arm_and_takeoff(self, TargetAltitude, ID):
		print "Arming Motors"
		print I
		setmode = rospy.ServiceProxy('/UAV'+str(ID)+'/mavros/set_mode', SetMode)
		setmode(216,"GUIDED")
		arm = rospy.ServiceProxy('/UAV'+str(ID)+'/mavros/cmd/arming', CommandBool)
		arm(True)
		print "Taking off!"
		takeoff = rospy.ServiceProxy('/UAV'+str(ID)+'/mavros/cmd/takeoff', CommandTOL)
		takeoff(0,0,0,0,TargetAltitude)

	def land(self):
		setmode = rospy.ServiceProxy('/UAV'+str(ID)+'/mavros/set_mode', SetMode)
		setmode(216,"LAND")

def distance(lat1, lat2, lon1, lon2): 
      
    # The math module contains a function named 
    # radians which converts from degrees to radians. 
    lon1 = radians(lon1) 
    lon2 = radians(lon2) 
    lat1 = radians(lat1) 
    lat2 = radians(lat2) 
       
    # Haversine formula  
    dlon = lon2 - lon1  
    dlat = lat2 - lat1 
    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
  
    c = 2 * asin(sqrt(a))  
     
    # Radius of earth in kilometers. Use 3956 for miles 
    r = 6371
       
    # calculate the result 
    return(c * r)

def distance_from_globalframe_to_localframe(pos):
	pos_new = pos-UAV_pos_initial
	return pos_new

def trajectory_generator(start, goal):
	start[0]=radians(start[0])
	start[1]=radians(start[1])
	goal[0]=radians(goal[0])
	goal[1]=radians(goal0[1])

	delta = goal[1]-start[1]
	bearing = atan2(sin(delta)*cos(goal[0]), cos(start[0])*sin(goal[0])-sin(start[0])*cos(goal[0])*cos(delta))

	lat_new = asin(sin(start[0])*cos(0.005/6371)+cos(start[0])*sin(0.005/6371)*cos(bearing))
	long_new = start[1]+atan2(sin(bearing)*sin(0.005/6371)*cos(start[0]), cos(0.005/6371)-sin(start[0])*sin(lat_new))

	dest[0]=lat_new
	dest[1]=long_new

vehicle=list()

for i in range(N):
    temp='v'+str(i+1)
    temp=Swarm_Bot(i+1)
    vehicle.append(temp)

UAV_pos_initial=list()
for i in range (N):
	UAV_pos_initial.append(vehicle[i].get_pos(i+1))

for i in range(N):
    vehicle[i].arm_and_takeoff(30,i+1)

dij=np.zeros((5,5))
d_eq = np.zeros((5,1))

goal=[28.778575,77.119601]
start = UAV_pos_initial[4]
########## Minus Formation ###############
const=0.06
dij[0][0]=0
dij[0][1]=const
dij[0][2]=const*2
dij[0][3]=const*3
dij[1][0]=const
dij[1][1]=0
dij[1][2]=const
dij[1][3]=const*2
dij[2][0]=const*2
dij[2][1]=const
dij[2][2]=0
dij[2][3]=const*2
dij[3][0]=const*3
dij[3][1]=const*2
dij[3][2]=const
dij[3][3]=0

d_eq[0][0] = const*sqrt(3)
d_eq[1][0] = const
d_eq[2][0] = const
d_eq[3][0] = const*sqrt(3)


a1=0.35
a=0.05
v_max=30
v_min=20
v=0
print vehicle
for j in range(1,T,dt):
    posL=trajectory_generator(start,goal)
    start=posL
    for i in range(N):
        pos1=vehicle[i].get_pos(i+1)
        pos_i=pos1
        print "Vehicle id", i+1

        d_L = distance(pos1[0],posL[0],pos1[1],posL[1])
        pos_temp = [0,0]

        for k in range(N):
            if i==k:
                continue
            pos2=vehicle[k].get_pos(i+1)
            d = distance(pos1[0],pos2[0],pos1[1],pos2[1])

            pos_temp = [pos_temp[0]-1*a*((pos1[0]-pos2[0])/d)*(1-dij[i][k]/d),pos_temp[1]-1*a*((pos1[1]-pos2[1])/d)*(1-dij[i][k]/d)]
            #v = v_max*(1-dij[i][k]/d)+v
            

        pos = [-1*a1*((pos1[0]-posL[0])/d_L)*(1-d_eq[i][0]/d_L) + pos_temp[0]+pos1[0],-1*a1*((pos1[1]-posL[1])/d_L)*(1-d_eq[i][0]/d_L) + pos_temp[1]+pos1[1]]
        #pos = [pos_temp[0]+pos1[0],pos_temp[1]+pos1[1]]
        #print pos
        
        
        v = v_max*(1-d_eq[i][0]/d_L)*1.5
        print v
        if v>v_max:
            v=v_max
        elif v<v_min:
            v=v_min

        print d_L
        s=s+v
        time.sleep(0.1)
        pos_converted = distance_from_globalframe_to_localframe(pos)
        vehicle[i].attitude_control(pos,v)


print "All Done"