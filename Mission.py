from math import radians, cos, sin, asin, sqrt 
import numpy as np
import time
import dronekit
from dronekit import connect, VehicleMode, LocationGlobal#import connect, VehicleMode, LocationGlobal, simple_goto
from pymavlink import mavutil
dt=1
T=100000

N=3
pos=[[38.146200,-76.428387],[38.145313,-76.429119],[38.149222,-76.429483],[38.150233,-76.430855]]

class swarm_bot:
    def __init__(self,n,pos,s):
        self.id=n
        self.string=str(s)
        self.pos=pos
        self.velocity = [0,0]
        print("Connecting to Vehicle id:",n)
        self.vehicle = connect(self.string)#, wait_ready=True)
        print("Connected")


    def get_pos(self):
        self.pos= [self.vehicle.location.global_frame.lat,self.vehicle.location.global_frame.lon]
        return self.pos


    def update_vel(self,v):
        self.velocity=v       
        velocity_x=v[0]
        velocity_y=v[1]
        velocity_z=0
        """
        Move vehicle in direction based on specified velocity vectors.
        """
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, # lat_int - X Position in WGS84 frame in 1e7 * meters
            0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
            0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
            # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
            velocity_x, # X velocity in NED frame in m/s
            velocity_y, # Y velocity in NED frame in m/s
            velocity_z, # Z velocity in NED frame in m/s
            0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        self.vehicle.send_mavlink(msg)
    

    def update_pos(self,pos):
        self.position=pos
        pos_x = pos[0]
        pos_y = pos[1]
        pos_z = 5

        a_location = LocationGlobal(pos_x, pos_y, pos_z)
        self.vehicle.simple_goto(a_location)

    def arm_and_takeoff(self, aTargetAltitude):
 
        print "Basic pre-arm checks",self.id

        while not self.vehicle.is_armable:
            print " Waiting for vehicle to initialise...",self.id
            time.sleep(1)

            
        print "Arming motors",self.id
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:      
            print " Waiting for arming...",self.id
            time.sleep(1)

        print "Taking off!",self.id
        self.vehicle.simple_takeoff(aTargetAltitude)

        while True:
            print " Altitude: ", self.vehicle.location.global_relative_frame.alt      
            if self.vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
                print "Reached target altitude",self.id
                break
            time.sleep(1)

    def land(self):
        self.vehicle.mode = VehicleMode("QLAND")



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



vehicle=list()
print N
for i in range(N):
    s ='127.0.0.1:'+str(14551+i*10)

    temp='v'+str(i+1)
    temp=swarm_bot(i+1,pos[i],s)
    vehicle.append(temp)

for i in range(N):
    vehicle[i].arm_and_takeoff(5)
    print"aniket"
    continue

"""
for i in range(N):
    for j in range(N):
        if j==i+1 or j==i-1 : 
            dij[i][j] = 5
        else:
            dij[i][j] = 
"""
##Multiply by a variable which equals 1 or 0
dij=np.zeros((3,3))
d_eq = np.zeros((3,1))

dij[0][0]=0
dij[0][1]=30
dij[0][2]=60
dij[1][0]=30
dij[1][1]=0
dij[1][2]=30
dij[2][0]=60
dij[2][1]=30
dij[2][2]=0

#goal=[28.768575,77.115601]
#goal2 = [28.748575,77.115601]
goal = [28.382192,77.116017]
goal2 = [28.362443,77.135230]
#vehicle[3].update_pos(goal)

d_eq[0][0] = 30
d_eq[1][0] = 60
d_eq[2][0] = 90

a = 1
print vehicle
for j in range(1,T,dt):
    posL = [28.754681+j*0.000100,77.116017]
    for i in range(N):
        v = [0,0]
        pos1=vehicle[i].get_pos()
        pos_i=pos1
        print "Vehicle id", i+1

        d_L = distance(pos1[0],posL[0],pos1[1],posL[1])
        pos_temp = [0,0]

        for k in range(N):
            if i==k:
                continue
            pos2=vehicle[k].get_pos()
            d = distance(pos1[0],pos2[0],pos1[1],pos2[1])

            pos_temp = [pos_temp[0]+1*a*((pos1[0]-pos2[0])/d)*(1-dij[i][k]/d),pos_temp[1]+1*a*((pos1[1]-pos2[1])/d)*(1-dij[i][k]/d)]

        pos = [1*a*((pos1[0]-posL[0])/d_L)*(1-d_eq[i][0]/d) + pos_temp[0]+pos1[0],1*a*((pos1[1]-posL[1])/d_L)*(1-d_eq[i][0]/d) + pos_temp[1]+pos1[1]]
        print pos
        vehicle[i].update_pos(pos)
    #time.sleep(1)

dij[0][0]=0
dij[0][1]=30
dij[0][2]=30
dij[1][0]=30
dij[1][1]=0
dij[1][2]=30
dij[2][0]=30
dij[2][1]=30
dij[2][2]=0

d_eq[0][0] = 30
d_eq[1][0] = 30*1.73
d_eq[2][0] = 30*1.73

vehicle[3].update_pos(goal2)

for j in range(1,T,dt):
    posL = vehicle[3].get_pos()
    for i in range(N):
        v = [0,0]
        pos1=vehicle[i].get_pos()
        pos_i=pos1
        print "Vehicle id", i+1

        d_L = distance(pos1[0],posL[0],pos1[1],posL[1])
        pos_temp = [0,0]

        for k in range(N):
            if i==k:
                continue
            pos2=vehicle[k].get_pos()
            d = distance(pos1[0],pos2[0],pos1[1],pos2[1])

            pos_temp = [pos_temp[0]+1*a*((pos1[0]-pos2[0])/d)*(1-dij[i][k]/d),pos_temp[1]+1*a*((pos1[1]-pos2[1])/d)*(1-dij[i][k]/d)]

        pos = [1*a*((pos1[0]-posL[0])/d_L)*(1-d_eq[i][0]/d) + pos_temp[0]+pos1[0],1*a*((pos1[1]-posL[1])/d_L)*(1-d_eq[i][0]/d) + pos_temp[1]+pos1[1]]
        print pos
        vehicle[i].update_pos(pos)


for i in range(N):
    land_goal = [28.362443,77.135230+i*400]
    vehicle[i].update_pos(land_goal)


while True:
    print "LAND1"
    vehicle[0].land()
    time.sleep(1)
    print "LAND2"
    vehicle[1].land()
    time.sleep(1)










##### IMPROVEMENT POINTS IN THE CODE ###############
"""
1. Give the goal waypoint at the end of the loop and the land command
2. Start by intialising 3 UAVs at start poisition 20 metres apart each
3. Remove the 4th UAV from the screen
4. Start with line formation and then switch to triangle formation
5. Land at the designated place
"""