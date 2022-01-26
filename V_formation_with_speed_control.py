from math import radians, cos, sin, asin, sqrt 
import numpy as np
import time
import dronekit
from dronekit import connect, VehicleMode, LocationGlobal#import connect, VehicleMode, LocationGlobal, simple_goto
from pymavlink import mavutil
import utm
dt=1
T=500000

N=2
pos=[[38.146200,-76.428387],[38.145313,-76.429119],[38.149222,-76.429483],[38.150233,-76.430855],[38.150233,-76.430855]]

class swarm_bot:
    def __init__(self,n,pos,s):
        self.id=n
        self.string=str(s)
        self.pos=pos
        self.velocity = [0,0]
        print("Connecting to Vehicle id:",n)
        self.vehicle = connect(self.string)#, wait_ready=True)1
        
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
    

    #def update_speed(self,speed):
    #    self.vehicle.groundspeed(speed)


    def update_pos(self,pos,v):
        self.position=pos
        pos_x = pos[0]
        pos_y = pos[1]
        pos_z = 30

        a_location = LocationGlobal(pos_x, pos_y, pos_z)
        self.vehicle.simple_goto(a_location,v,v)

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

print N+1
for i in range(N+1):
    s ='127.0.0.1:'+str(14551+i*10)
    temp='v'+str(i+1)
    temp=swarm_bot(i+1,pos[i],s)
    vehicle.append(temp)




for i in range(N+1):
    vehicle[i].arm_and_takeoff(30)

    

"""
for i in range(N):
    for j in range(N):
        if j==i+1 or j==i-1 : 
            dij[i][j] = 5
        else:
            dij[i][j] = 
"""
##Multiply by a variable which equals 1 or 0

dij=np.zeros((5,5))
d_eq = np.zeros((5,1))

"""
const=0.06
dij[0][0]=0
dij[0][1]=const*3
dij[1][0]=const*3
dij[1][1]=0

goal=[28.778575,77.119601]
goal2=[28.780634,77.094713]
goal3=[28.762276,77.092283]
vehicle[N].update_pos(goal,25)

d_eq[0][0] = const
d_eq[1][0] = const

"""
"""
########## V formation #################
const=0.06
dij[0][0]=0
dij[0][1]=const
dij[0][2]=const
dij[0][3]=const*sqrt(3)
dij[1][0]=const
dij[1][1]=0
dij[1][2]=const*sqrt(3)
dij[1][3]=const
dij[2][0]=const
dij[2][1]=const*sqrt(3)
dij[2][2]=0
dij[2][3]=const*2
dij[3][0]=const*sqrt(3)
dij[3][1]=const
dij[3][2]=const*2
dij[3][3]=0

d_eq[0][0] = const
d_eq[1][0] = const
d_eq[2][0] = const*2
d_eq[3][0] = const*2
"""

goal=[28.778575,77.119601]
goal2=[28.780634,77.094713]
goal3=[28.762276,77.092283]
vehicle[N].update_pos(goal3,25)

"""
########## Line Formation ###############
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

d_eq[0][0] = const
d_eq[1][0] = const*2
d_eq[2][0] = const*3
d_eq[3][0] = const*4
"""

"""
########## Plus Formation ################
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

d_eq[0][0] = const
d_eq[1][0] = const*2
d_eq[2][0] = const*3
d_eq[3][0] = const*4
"""


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

d_eq[0][0] = const
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
    posL = [28.762276,77.092283]
    s=0
    for i in range(N):
        #vel = [0,0]
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

            pos_temp = [pos_temp[0]-1*a*((pos1[0]-pos2[0])/d)*(1-dij[i][k]/d),pos_temp[1]-1*a*((pos1[1]-pos2[1])/d)*(1-dij[i][k]/d)]
            #v = v_max*(1-dij[i][k]/d)+v
            

        pos = [-1*a1*((pos1[0]-posL[0])/d_L)*(1-d_eq[i][0]/d_L) + pos_temp[0]+pos1[0],-1*a1*((pos1[1]-posL[1])/d_L)*(1-d_eq[i][0]/d_L) + pos_temp[1]+pos1[1]]
        #pos = [pos_temp[0]+pos1[0],pos_temp[1]+pos1[1]]
        #print pos1
        
        v = v_max*(1-d_eq[i][0]/d_L)*1.5
        print v
        if v>v_max:
            v=v_max
        elif v<v_min:
            v=v_min

        print d_L
        s=s+v
        time.sleep(0.1)
        #vehicle[i].update_speed(v)
        vehicle[i].update_pos(pos,v)
    #time.sleep(1)

    #vehicle[N].update_pos(goal3,s/N)

while True:
    print "LAND1"
    vehicle[0].land()
    time.sleep(1)
    print "LAND2"
    vehicle[1].land()
    time.sleep(1)




"""
dij=np.zeros((5,5))
d_eq = np.zeros((5,1))

dij[0][0]=0#0
dij[0][1]=0#30
dij[0][2]=0#30
dij[0][3]=0#50#1.9
dij[1][0]=0#30
dij[1][1]=0#0
dij[1][2]=0#51.9
dij[1][3]=0#30
dij[2][0]=0#30
dij[2][1]=0#51.9
dij[2][2]=0#0
dij[2][3]=0#60
dij[3][0]=0#51.9
dij[3][1]=0#30
dij[3][2]=0#60
dij[3][3]=0#0

goal=[28.778575,77.119601]
vehicle[N].update_pos(goal)

d_eq[0][0] = 30
d_eq[1][0] = 30
d_eq[2][0] = 60
d_eq[3][0] = 60


#d_eq[2][0] = 90

a = 1
print vehicle
for j in range(1,T,dt):
    posL = vehicle[N].get_pos()
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
        time.sleep(0.1)
        vehicle[i].update_pos(pos)

"""