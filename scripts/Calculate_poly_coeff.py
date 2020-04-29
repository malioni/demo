import numpy as np

# Initial and final conditions (s(0),s(T),sd(0),sd(T),sdd(0),sdd(T))
s = np.array([[-1.5],[0.0],[0.0],[0.0],[0.0],[0.0]])
t = 0.0
T = 4.0

X = np.array([[0.,0.,0.,0.,0.,1.],[T**5,T**4,T**3,T**2,T,1.],[0.,0.,0.,0.,1.,0.],[5.*T**4,4.*T**3,3.*T**2,2.*T,1.,0.],[0.,0.,0.,2.,0.,0.],[20.*T**3,12.*T**2,6.*T,2.,0.,0.]])
print(np.linalg.inv(X)@s)