import numpy as np
import matplotlib.pyplot as plt
import actionlib
import rospy
from gr_action_msgs.msg import PolyFitRowAction, PolyFitRowActionGoal
import time
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class Action(object):
    def __init__(self):
        rospy.init_node("polyfit_server")
        self.gpath_pub = rospy.Publisher("global", Path)
        self.lpath_pub = rospy.Publisher("local", Path)

        self._as = actionlib.SimpleActionServer('polyfit_action', PolyFitRowAction, execute_cb=self.myexecution_cb, auto_start = False)
        self.dt = 0.1
        self.x = np.array([0,2,4])
        self.y = np.array([0,0,0])
        self.thetas = np.array([0,0,0])
        self.final = [self.x[-1],self.y[-1], 0]
        self.nmotion = 2
        self.ac = 0.25
        self.max_vel = 1.0
        self.v = 0.0
        self.s = 0.0
        self.max_steer = np.pi/6
        self.trajectory = []
        self.motions = []
        self.vels =[]
        self._as.start()
        print "server started"

    def myexecution_cb(self, goal):
        print "received"
        #plt.clf()
        self.motions = []
        self.vels =[]
        self.x = []
        self.y = []
        self.thetas = []
        self.trajectory = []
        self.v = 0

        for xi,yi,zi in zip(goal.x, goal.y,goal.yaw):
            self.x.append(xi)
            self.y.append(yi)
            self.thetas.append(zi)

        if len(self.x) < 4:
            rospy.loginfo("avoiding")
            time.sleep(5)
            self._as.set_succeeded()
            return

        self.calculate()
        self.setStart()

        for i in range(500):
            print "run ", i
            self.run()
            if self.complete():
                break
        self.plotTrajectory()

        print "MOTIONS ", self.motions
        print "vels ", self.vels

        time.sleep(3)
        self._as.set_succeeded()

    def cost2goal(self, p):
        return np.sqrt(np.power(self.final[0] - p[0],2)+np.power(self.final[1] - p[1],2))

    def complete(self):
        #print self.final, self.startx, self.starty
        return np.sqrt(np.power(self.final[0] - self.startx,2)+np.power(self.final[1] - self.starty,2)) < 0.15 # and np.fabs(self.starttheta - self.final[2]) < 0.2

    def setStart(self):
        self.startx = self.x[0]
        self.starty = self.y[0]
        self.starttheta = self.thetas[0]
        self.final = [self.x[-1],self.y[-1], self.thetas[-1]]


    def predict(self, motion,x,y):
        L = 1.0
        psi = self.starttheta

        time2stop = np.fabs(self.v*self.ac+0.1)
        #print "TIME2 stop", np.fabs(self.v/time2stop)
        if self.v > 0.0000:
            self.stop = self.v/time2stop
        else:
            self.stop = 0.0

        if self.cost2goal([x, y]) <= np.fabs(self.v/time2stop):
            v1 = self.v - (self.ac * self.dt)
            v1 = max(0.0, v1)
            psi1 = psi+v1/L * self.s * self.dt
            return x + v1*np.cos(psi)*self.dt, y + v1*np.sin(psi)* self.dt, psi1, v1, self.s


        #v+ s=
        if motion == 0:
            v1 = self.v + (self.ac * self.dt)
            v1 = min(self.max_vel, v1)
            #BRUTE FORCE STOP
            time2stop = np.fabs(self.v*self.ac+0.1)
            psi1 = psi+v1/L * self.s * self.dt
            return x + v1*np.cos(psi)*self.dt, y + v1*np.sin(psi)* self.dt, psi1, v1, self.s

        #v- s=
        if motion == 1:
            v1 = self.v - (self.ac * self.dt)
            v1 = max(0.0, v1)
            #psi1 = self.s+psi+v1/L * np.tan(self.s)
            psi1 = psi+v1/L * self.s * self.dt
            return x + v1*np.cos(psi)*self.dt, y + v1*np.sin(psi)* self.dt, psi1, v1, self.s
        #v= s=
        if motion == 2:
            #psi1 = self.s+psi+self.v/L * np.tan(self.s)
            psi1 = psi+self.v/L * self.s * self.dt
            return x + self.v*np.cos(psi1)*self.dt, y + self.v*np.sin(psi1)* self.dt,psi1, self.v, self.s

        #v+ s+
        if motion == 3:
            v1 = self.v + (self.ac * self.dt)
            v1 = min(self.max_vel, v1)
            s1 = self.s +  0.1
            s1 = min(s1,self.max_steer)
            psi1 = psi + v1/L * self.dt *s1
            return x + v1*np.cos(psi)*self.dt, y + v1*np.sin(psi)* self.dt,psi1, v1, s1
        #v- s+
        if motion == 4:
            v1 = self.v - (self.ac * self.dt)
            v1 = min(self.max_vel, v1)
            s1 = self.s+0.1
            s1 = min(s1,self.max_steer)
            psi1 = psi + s1 * v1/L * self.dt
            return x + v1*np.cos(psi)*self.dt, y + v1*np.sin(psi)* self.dt, psi1, v1, s1

        #v= s+
        if motion == 5:
            s1 = self.s + 0.1
            s1 = min(s1,self.max_steer)
            psi1 = psi + s1* self.v/L * self.dt
            return x + self.v*np.cos(psi)*self.dt, y + self.v*np.sin(psi)* self.dt, psi1, self.v, s1

        #v+ s-
        if motion == 6:
            v1 = self.v + (self.ac * self.dt)
            v1 = min(self.max_vel, v1)
            s1 = self.s - 0.1
            s1 = max(-self.max_steer,s1)
            psi1 = psi + v1/L * self.dt * s1
            return x + v1*np.cos(psi)*self.dt, y + v1*np.sin(psi)* self.dt, psi1, v1,s1

        #v- s-
        if motion == 7:
            v1 = self.v - (self.ac * self.dt)
            v1 = min(self.max_vel, v1)
            s1 = self.s - 0.1
            s1 = max(0,s1)
            psi1 = psi + v1/L * self.dt * s1
            return x + v1*np.cos(psi)*self.dt, y + v1*np.sin(psi)* self.dt, psi1, v1,s1

        #v= s-
        if motion == 8:
            s1 = self.s - 0.1
            s1 = max(-self.max_steer,s1)
            psi1 = psi+ self.v/L * self.dt * s1
            return x + self.v*np.cos(psi)*self.dt, y + self.v*np.sin(psi)* self.dt, psi1, self.v,s1


    def apply(self, motion,states):
        L = 1.0
        if self.cost2goal([self.startx, self.starty]) <= self.stop:
            motion = 1
            print "apply motion ", motion
        new_state = states[motion]
        self.motions.append(motion)
        self.vels.append(new_state[3])
        self.startx = new_state[0]
        self.starty = new_state[1]
        self.v = new_state[3]
        self.s = new_state[4]
        self.starttheta = new_state[2]
        print "new state ", new_state


    def evalMotion(self,x,y, theta):
        #plt.scatter(x, y, c='b', s=20.0)
        #print "costs ", 20*np.fabs(y - np.polyval(self.coeff, x)) , 0.25*self.cost2goal([x, y]) , 20.0 *np.fabs(theta-self.final[2])
        cost = 5.0*self.cost2goal([x, y]) + 100.0 *int(self.stop > self.cost2goal([x, y]))# 0.5*(self.max_vel-self.v)

        #cost = 0.50*np.fabs(x - np.polyval(self.coeff, y)) + 5.0*self.cost2goal([x, y]) + 0.20 *np.fabs(theta-self.final[2]) + 100.0 *int(self.stop > self.cost2goal([x, y]))# 0.5*(self.max_vel-self.v)
        return cost

    def evalStep(self):
        #plt.scatter(self.startx, self.starty, c='r', s=150.0)
        self.trajectory.append([self.startx, self.starty, self.starttheta])
        return self.evalMotion(self.startx, self.starty, self.starttheta)

    def plotTrajectory(self):
        p = Path()
        p.header.frame_id = "map"
        for xi,yi,zi in self.trajectory:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x = xi
            ps.pose.position.y = yi
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)

        self.lpath_pub.publish(p)
        plt.plot(np.array(self.trajectory)[:,0], np.array(self.trajectory)[:,1], c='g')
        plt.draw()
        #time.sleep(100)
        #plt.show()

    def run(self):
        scores = np.zeros(self.nmotion)
        x = self.startx
        y = self.starty
        theta = self.starttheta

        states = np.zeros((self.nmotion, 5))

        for i in range(self.nmotion):
            #print " motion prediction ", i
            xs,ys,ts,vs,ss = self.predict(i,x,y)
            states[i] = [xs,ys,ts,vs,ss]
            scores[i] += self.evalMotion(xs,ys,ts)

        #print ("states",states)

        action_indx = np.argmin(scores/np.sum(scores))
        #print (scores/np.sum(scores), action_indx)
        self.apply(action_indx, states)
        print "cost of step ", self.evalStep()

    def drawArrow(self,A, B):
        plt.arrow(A[0], A[1], B[0] - A[0], B[1] - A[1],
              head_width=1.0, length_includes_head=False)

    def calculate(self):
        s0 = self.thetas[0]#np.pi/2
        sf = self.thetas[-1]
        self.coeff = (np.polyfit(self.y,self.x,3))
        y1 = np.linspace(np.min(self.y),np.max(self.y),10)
        x1 = np.polyval(self.coeff,y1)
        ang3 = []
        for i in range(len(x1)-1):
            ang3.append(np.arctan2(y1[i+1]-y1[i],x1[i+1]-x1[i]))

        ang1 = np.arctan2(y1,x1)
        ang2 = np.linspace(s0,sf, len(x1))

        p = Path()
        p.header.frame_id = "map"
        for xi,yi in zip(x1,y1):
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x = xi
            ps.pose.position.y = yi
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)

        self.gpath_pub.publish(p)

        #print ( ang1, "angulars")
        #plt.plot(x1,np.asarray(y1), c="b")
        #plt.scatter(self.x,self.y, c="b")

        #for i in range(len(ang1)-1):
        #    self.drawArrow([x1[i],y1[i]], [x1[i+1], y1[i+1]])


if __name__ == '__main__':
    print "main"
    a = Action()
    rospy.spin()

    #a.calculate()
    #a.setStart()
    #for i in range(1000):
    #    a.run()
    #    if a.complete():
    #        print "MOTION COMPLETED BEFORE TIME"
    #        break
    #        print ("STEP SCORE ", a.evalStep())
#a.plotTrajectory()
#print (a.trajectory)
