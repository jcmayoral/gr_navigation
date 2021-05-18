import numpy as np
import matplotlib.pyplot as plt


class Action:
    def __init__(self):
        self.x = np.array([0,2,4])
        self.y = np.array([0,6,0])
        self.final = [self.x[-1],self.y[-1]]
        self.nmotion = 8
        self.trajectory = []

    def cost2goal(self, p):
        return np.sqrt(np.power(self.final[0] - p[0],2)+np.power(self.final[1] - p[1],2))

    def setStart(self):
        self.startx = self.x[0]
        self.starty = self.y[0]

    def predict(self, motion,x,y):
        if motion == 0:
            return x + 0.1, y
        if motion == 1:
            return x, y + 0.1
        if motion == 2:
            return x + 0.1, y + 0.1
        if motion == 3:
            return x+0.1, y-0.1
        if motion == 4:
            return x - 0.1, y+0.1
        if motion == 5:
            return x-0.1, y-0.1
        if motion == 6:
            return x-0.1, y
        if motion == 7:
            return x, y-0.1

    def apply(self, motion):
        print ("Applying", motion, self.startx, self.starty)
        if motion == 0:
            self.startx += 0.1
        if motion == 1:
            self.starty += 0.1
        if motion == 2:
            self.startx += 0.1
            self.starty +=0.1
        if motion == 3:
            self.startx +=0.1
            self.starty -=0.1
        if motion == 4:
            self.startx -= 0.1
            self.starty += 0.1
        if motion == 5:
            self.startx -=0.1
            self.starty -=0.1
        if motion == 6:
            self.startx -=0.1
        if motion == 7:
            self.starty -=0.1
        print ("End", motion, self.startx, self.starty)


    def evalMotion(self,x,y, t):
        #plt.scatter(x, y, c='b', s=20.0*t)
        return 0.2*np.fabs(y - np.polyval(self.coeff, x)) + 100.5*self.cost2goal([x, y])

    def evalStep(self):
        #plt.scatter(self.startx, self.starty, c='r', s=150.0)
        self.trajectory.append([self.startx, self.starty])
        return 0.2*np.fabs(self.starty - np.polyval(self.coeff, self.startx)) + 0.5*self.cost2goal([self.startx, self.starty])

    def plotTrajectory(self):
        print (np.array(self.trajectory)[:,0].shape)
        plt.plot(np.array(self.trajectory)[:,0], np.array(self.trajectory)[:,1], c='g')

    def run(self):
        scores = np.zeros(self.nmotion)
        x = self.startx
        y = self.starty

        for i in range(self.nmotion):
            xs,ys = self.predict(i,x,y)
            scores[i] += self.evalMotion(xs,ys,10)

        action_indx = np.argmin(scores/np.sum(scores))
        print (scores/np.sum(scores), action_indx)
        self.apply(action_indx)
        print self.evalStep()

    def drawArr150ow(self,A, B):
        plt.arrow(A[0], A[1], B[0] - A[0], B[1] - A[1],
              head_width=1.0, length_includes_head=False)

    def calculate(self):
        s0 = np.pi/2
        sf = 0
        self.coeff = (np.polyfit(self.x,self.y,4))
        x1 = np.linspace(0,np.max(self.x),10)
        y1 = np.polyval(self.coeff,x1)
        ang3 = []
        for i in range(len(x1)-1):
            ang3.append(np.arctan2(y1[i+1]-y1[i],x1[i+1]-x1[i]))

        ang1 = np.arctan2(y1,x1)
        ang2 = np.linspace(s0,sf, len(x1))
        #print ( ang1, "angulars")
        plt.plot(x1,np.asarray(y1))
        plt.scatter(self.x,self.y)

        #for i in range(len(ang1)-1):
        #    self.drawArrow([x1[i],y1[i]], [x1[i+1], y1[i+1]])

a = Action()
a.calculate()
a.setStart()
for i in range(50):
    a.run()
    print ("STEP SCORE ", a.evalStep())
a.plotTrajectory()
plt.show()
