#!/usr/bin/env python
print "Lab 5"
import roslib
roslib.load_manifest('lab5')
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import rospkg
import threading
rospack=rospkg.RosPack()
path=rospack.get_path('lab5')
path=path+'/data/map.txt'

#FLAGS
angle=0
trans=0
tempgoal=0
reached=0



def cell(x,y):
	x=int(x+9)
	y=int(y+10)
	return x,y

def coord(x,y):
	x=float(x)
	y=float(y)
	x=x-9+0.5
	y=y-10
	return x,y

def parentcheck(x,y,temp):
	n=len(d.child)
	for i in range(0,n):
		if d.child[i] in temp:
			d1=d.score[i]
			d2=d.vscore[d.count]+1
			if d2<d1:
				d.parent[i]=[x,y]

		

		
def calcopen(current,opn):
	x=current[0]
	y=current[1]
	temp=[]
	temp.append([x+0,y+1])
	temp.append([x+1,y+0])
	temp.append([x+0,y-1])
	temp.append([x-1,y+0])
	for i in range(0,4):
		p=temp[i]
		if 0<=p[0]<=17 and 0<=p[1]<=19 and p not in d.obstacles and p not in d.visited and p not in opn:
			opn.append(p)
			d.child.append(p)
			d.parent.append([x,y])
			d.score.append(d.vscore[d.count]+1) 

	parentcheck(x,y,temp)
	return opn

def manhattan(x1,y1,x2,y2):
	x=np.absolute(x1-x2)
	y=np.absolute(y1-y2)
	return x+y

def euclidean(xi,yi,fx,fy):
	dist=math.sqrt((xi-fx)**2 + (yi-fy)**2)
	return dist


def refine():
	A=[]
	previous=[0,0]
	for i in d.visited:
		if i!=previous:
			A.append(i)
			previous=i
	return A		

def recalc(xi,yi,xf,yf,factor):	
	#A* Implementation
	visited=[]
	opn=[]
	temp=[xi,yi]
	visited.append(temp)
	d.visited=visited
	del d.vscore[:]
	del d.child[:]
	del d.score[:]
	del d.parent[:]
	d.count=0
	d.vscore.append(0)

	count=0
	while(count<600):
		if [xf,yf] in visited:
			print "Goal reached"
			break
		else:
			
			current=visited[count]
			opn=calcopen(current,opn)
			val=[]
			for i in opn:
				j=d.child.index(i)

				mc=d.score[j]*factor
				h=manhattan(i[0],i[1],xf,yf)
				tempval=mc+h
				val.append(tempval)

			mincost=min(val)
			n=len(val)
			for i in range(0,n):
				if val[i]==mincost:
					break

			temp=opn[i]
			#print temp
			visited.append(temp)
			d.visited=visited
			d.vscore.append(d.vscore[count]+1)
			opn.remove(temp)
			count=count+1
			d.count=d.count+1

	#d.visited=refine()


	nx=len(d.child)
	finalgoal=d.child[nx-1]
	par=[0,0]
	newvisited=[]
	while(par!=[xi,yi]):
		newvisited.append(finalgoal)
		parind=d.child.index(finalgoal)
		par=d.parent[parind]
		if par!=[xi,yi]:
			childind=d.child.index(par)
		finalgoal=par


	nx=len(newvisited)
	newvisited2=[]
	nx=nx-1
	while(nx>=0):
		newvisited2.append(newvisited[nx])
		nx=nx-1

	d.visited=newvisited2
	print d.visited
	d.counter=0

def getfinalvalues(msg):
	if msg!=None:
		msg=str(msg)
  		msg=msg[6:]
  		mylist=msg.split(',')
  		goalx=float(mylist[0])
  		goaly=float(mylist[1])
  		return goalx,goaly
  	else:
  		return d.xf,d.yf	

def getodomvalues(odom_data):
	msg=odom_data
	msg.header.stamp = rospy.Time.now()
	pos = (msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z)
	xi=msg.pose.pose.position.x
	yi=msg.pose.pose.position.y
	return xi,yi

def position(odom_data):
	global angle
	global trans
	global tempgoal
	global reached

	xf=rospy.get_param('goalx',float(4.5))
	yf=rospy.get_param('goaly',float(9.0))
	if (xf!=d.xf or yf!=d.yf):
		reached=0
		d.xf=xf
		d.yf=yf
		xf,yf=cell(xf,yf)
		print xf,yf
		bx=manhattan(d.xi,d.yi,xf,yf)
		factor=float(bx/36)
		recalc(d.xi,d.yi,xf,yf,factor)
		d.counter=0


	
	if reached==0:
		goal=d.visited[d.counter]
		fx=goal[0]
		fy=goal[1]
		d.xi=fx
		d.yi=fy
		fx,fy=coord(fx,fy)

		pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
		msg=odom_data
		msg.header.stamp = rospy.Time.now()
		pos = (msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z)
		ori = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)

		if angle==0 and trans==0 and tempgoal==0 and reached==0:

			(roll, pitch, yaw) = euler_from_quaternion([ori[0],ori[1],ori[2],ori[3]])	
			ra=yaw

			xdiff=fx-pos[0]
			ydiff=fy-pos[1]
			adiff=math.atan2(ydiff,xdiff)
			offset=adiff-ra

			if (-.01 < offset < .01):
				angle=1
			else:
				pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,offset)))

		if angle==1 and trans==0 and tempgoal==0 and reached==0:
			xi=msg.pose.pose.position.x
			yi=msg.pose.pose.position.y
			offset=euclidean(xi,yi,fx,fy)

			if (-.08 < offset < .08):
				angle=0
				trans=0
				tempgoal=0
				d.counter=d.counter+1
				if d.counter==len(d.visited):
					reached=1
			else:	
				pub.publish(Twist(Vector3(1,0,0),Vector3(0,0,0)))
				


class data(object):
	def __init__(self):
		self.name='data'
		self.grid=[]
		self.xf=0
		self.yf=0
		self.xi=0
		self.yi=0
		self.obstacles=[]
		self.visited=[]
		self.counter=0
		self.parent=[]
		self.child=[]
		self.score=[]
		self.vscore=[]
		self.count=0
		
		

d=data()
A=[]
mapfile=open(path,'r')
for i in mapfile:
	temp=[]
	for j in i:
		if j=='0' or j=='1':
			temp.append(int(j))
	A.append(temp)

A=np.array(A)
A=A.transpose()	

obstacles=[]
for i in range(0,18):
	for j in range(0,20):
		if A[i][j]==1:
			temp=[i,np.absolute(j-19)]
			obstacles.append(temp)	

d.grid=A
d.obstacles=obstacles
	

#initial position is (-8,-2) and default goal is (4.5,9)
xi=-8
yi=-2
xi,yi=cell(xi,yi)


goalx=rospy.get_param('goalx',float(4.5))
goaly=rospy.get_param('goaly',float(9.0))


xf=goalx
yf=goaly
d.xf=xf
d.yf=yf
xf,yf=cell(xf,yf)


#A* Implementation
visited=[]
opn=[]
temp=[xi,yi]
visited.append(temp)
d.visited=visited
d.vscore.append(0)

count=0
while(count<600):
	if [xf,yf] in visited:
		print "Goal reached"
		break
	else:
		
		current=visited[count]
		opn=calcopen(current,opn)
		val=[]
		for i in opn:
			j=d.child.index(i)

			mc=d.score[j]*0.8
			h=manhattan(i[0],i[1],xf,yf)
			tempval=mc+h
			val.append(tempval)

		mincost=min(val)
		n=len(val)
		for i in range(0,n):
			if val[i]==mincost:
				break

		temp=opn[i]
		#print temp
		visited.append(temp)
		d.visited=visited
		d.vscore.append(d.vscore[count]+1)
		opn.remove(temp)
		count=count+1
		d.count=d.count+1

#d.visited=refine()


nx=len(d.child)
finalgoal=d.child[nx-1]
par=[0,0]
newvisited=[]
while(par!=[1,8]):
	newvisited.append(finalgoal)
	parind=d.child.index(finalgoal)
	par=d.parent[parind]
	if par!=[1,8]:
		childind=d.child.index(par)
	finalgoal=par


nx=len(newvisited)
newvisited2=[]
nx=nx-1
while(nx>=0):
	newvisited2.append(newvisited[nx])
	nx=nx-1

d.visited=newvisited2
print d.visited


if __name__ =="__main__":
	try:
		rospy.init_node('lab5',anonymous=True)

		
		sub=rospy.Subscriber('base_pose_ground_truth', Odometry,position)

		
				
		
		rate = rospy.Rate(10)
		

	except rospy.ROSInterruptException:
        	pass
	
	rospy.spin()			
