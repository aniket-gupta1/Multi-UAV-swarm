from geopy.point import Point
from geopy.distance import distance
import utm
import math



#-p1 = Point(28.75316080,77.11414754)
#-p2 = Point(28.75583333,77.11694444)
#-p3 = Point(28.75333333,77.12000000)
#-p4 = Point(28.75083333,77.11722222)
#p3 = Point(28.75472222,77.11861111)
#p4 = Point(28.75194444,77.11555556)
#-p5 = Point(28.752460468, 77.1182377713)
#-p6 = Point(28.754335468, 77.1181336232)
#-p7 = Point(28.7542060954, 77.115931753)
#-p8 = Point(28.7523310952, 77.116035941)
#p3_a = (28.75472222,77.11861111)
#p4_a = (28.75194444,77.11555556)

#print(distance(p1,p2).meters)
#print(distance(p2,p3).meters)
#print(distance(p3,p4).meters)
#print(distance(p4,p1).meters)


utm_p1=utm.from_latlon(28.75316080,77.11414754) #0,0
utm_p2=utm.from_latlon(28.75583333,77.11694444) #0,400
utm_p3=utm.from_latlon(28.75333333,77.12000000) #400,400
utm_p4=utm.from_latlon(28.75083333,77.11722222) #400,0
utm_p5=utm.from_latlon(28.752460468, 77.1182377713)
utm_p6=utm.from_latlon(28.754335468, 77.1181336232)
utm_p7=utm.from_latlon(28.7542060954, 77.115931753)
utm_p8=utm.from_latlon(28.7523310952, 77.116035941)

utm_p9=utm.from_latlon(28.75266516007704, 77.11638554917288)
#print utm_p1
#print utm_p2
#print utm_p3
#print utm_p4

c=utm_p1[0]
f=utm_p1[1]
b=(utm_p2[0]-c)/400
e=(utm_p2[1]-f)/400
a=(utm_p4[0]-c)/400
d=(utm_p4[1]-f)/400
"""
rows=200
cols=200
est=utm_p9[0]
nrt=utm_p9[1]
x = int(( (e*est - b*nrt) - (c*e - f*b) )/(e*a - b*d))
y = int(( (d*est - a*nrt) - (c*d - a*f) )/(d*b - a*e))
print x,y
"""
x=390
y=10


print utm_p1[2], utm_p1[3]

est=a*x+b*y+c
nrt=d*x+e*y+f
#print (est, nrt, 43,"R")
d= utm.to_latlon(est, nrt, utm_p1[2], utm_p1[3])
print d
#p5=Point(d[0],d[1])
#print(distance(p3,p5).meters)
