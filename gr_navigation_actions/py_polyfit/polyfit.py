import numpy as np
import matplotlib.pyplot as plt

x = np.array([1,2,3,4])
y = np.array([0,0,0,0])
coeff = (np.polyfit(x,y,4))
print (coeff, coeff.shape)
x1 = np.linspace(0,np.max(x),10)
y1 = []
for xi in x1:
    y1.append(np.power(xi,4)*coeff[0] + np.power(xi,3)*coeff[1] + np.power(xi,2)*coeff[2] + xi *coeff[3] + coeff[4])

print (x1)
print (np.asarray(y1))
plt.plot(x1,np.asarray(y1))
plt.scatter(x,y)
plt.show()
