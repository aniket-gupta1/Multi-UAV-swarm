from math import radians, cos, sin, asin, sqrt 
import numpy as np
import time
import dronekit
from dronekit import connect, VehicleMode, LocationGlobal#import connect, VehicleMode, LocationGlobal, simple_goto
from pymavlink import mavutil
dt=1
T=50000

N=2
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
    	self.vehicle.mode = VehicleMode("LAND")



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


print vehicle
for j in range(1,T,dt):
    for i in range(N):
        v = [0,0]
        pos1=vehicle[i].get_pos()
        print "Vehicle id", i+1

        for k in range(N):
        	if i==k:
        		continue
        	pos2=vehicle[k].get_pos()
        	d = distance(pos1[0],pos2[0],pos1[1],pos2[1])
        	print d
        	#w=(d-0.001)*5000/(d-0.0005)
        	w=5000
        	e=[pos2[0]-pos1[0],pos2[1]-pos1[1]]
        	pos=[pos2[0]+e[0],pos2[1]+e[1]]


        	if d<0.01:
        		v=[0,0]
        
        print pos
        vehicle[i].update_pos(pos)
    #time.sleep(1)

while True:
	print "LAND1"
	vehicle[0].land()
	time.sleep(1)
	print "LAND2"
	vehicle[1].land()
	time.sleep(1)

