import numpy as np
import matplotlib.pyplot as plt


class Action:
    def __init__(self):
        self.x = np.array([0,2,4])
        self.y = np.array([0,6,0])

    def setStart(self):
        self.startx = 0
        self.starty = 0

    def step(self):
        self.startx += 0.1
        self.starty += 0.1

    def evalStep(self):
        print self.startx, self.starty
        plt.scatter(self.startx, self.starty, c='b')
        return np.fabs(self.starty - np.polyval(self.coeff, self.startx))

    def run(self):
        print self.evalStep()
        self.step()

    def drawArrow(self,A, B):
        plt.arrow(A[0], A[1], B[0] - A[0], B[1] - A[1],
              head_width=0.1, length_includes_head=True)

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

        for i in range(len(ang1)-1):
            self.drawArrow([x1[i],y1[i]], [x1[i+1], y1[i+1]])

a = Action()
a.calculate()
a.setStart()
for i in range(20):
    a.run()
plt.show()
