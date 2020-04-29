import numpy as np

def trajectory_planner(time, a=2.0, b=1.0, h=1.5,c=0.5):
    # a is the width of the trajectory. a = 2.0 means that the path will
    # be 2 meters wide and 4 meters in length

    # b is the change in height of the trajectory. b = 1.0 means that
    # the trajectory will go 1 meter above and 1 meter below the
    # desired height (h)

    # h is the middle height value of the trajectory

    # c determines the speed of the trajectory. c = 1.0 means that the
    # trajectory will go through one rotation in 2*pi seconds. c = 0.5 
    # means the trajectory will take 4*pi seconds to go through one
    # rotation

    # Lift up to needed height and hover there
    if time < 4.0:
        # These A,B,C coefficients were calculated by hand and depend on the ascending time
        # being 4 seconds. If time is changed, these coefficients need to be recalculated
        # A script is provided under demo/scripts, if needed
        A = -0.00878906
        B = 0.08789063
        C = -0.234375
        pos = np.array([0.0,0.0,A*time**5+B*time**4+C*time**3])
        vel = np.array([0.0,0.0,5.*A*time**4+4.*B*time**3+3.*C*time**2])
        head = 0.0
        head_d = 0.0
        accel = np.array([0.0,0.0,20.*A*time**3+12.*B*time**2+6.*C*time])
    # Hover at the desired height until time is 2*pi/c
    elif time < 2.0*np.pi/c:
        pos = np.array([0.0,0.0,-h])
        vel = np.array([0.0,0.0,0.0])
        head = 0.0
        head_d = 0.0
        accel = np.array([0.0,0.0,0.0])
    elif time < 8.0*np.pi/c: # Three rounds of slanted figure 8
        # final trajectory, yaw is left commented for reference
        pos = np.array([a/2*np.sin(2*c*time),a*np.sin(c*time),-h+b*np.sin(c*time)])
        vel = np.array([a*c*np.cos(2*c*time),a*c*np.cos(c*time),b*c*np.cos(c*time)])
        head = 0.0#np.arctan2(vel[1],vel[0])
        head_d = 0.0#(-c*np.sin(c*time)*np.cos(2*c*time)+2*c*np.sin(2*c*time)*np.cos(c*time))/((np.cos(c*time))**2 + (np.cos(2*c*time))**2)
        accel = np.array([-2*c**2*a*np.sin(2*c*time),-a*c**2*np.sin(c*time),-b*c**2*np.sin(c*time)])
    elif time < 8.0*np.pi/c + 6.: # Hover before descending
        pos = np.array([0.0,0.0,-h])
        vel = np.array([0.0,0.0,0.0])
        head = 0.0
        head_d = 0.0
        accel = np.array([0.0,0.0,0.0])
    elif time < 8.0*np.pi/c + 10.: # Descend
        # Same idea with the coefficients as mentioned above
        A = 0.00878906
        B = -0.08789063
        C = 0.234375
        pos = np.array([0.0,0.0,A*time**5+B*time**4+C*time**3-h])
        vel = np.array([0.0,0.0,5.*A*time**4+4.*B*time**3+3.*C*time**2])
        head = 0.0
        head_d = 0.0
        accel = np.array([0.0,0.0,20.*A*time**3+12.*B*time**2+6.*C*time])
    else: # Zero commands
        pos = np.array([0.0,0.0,0.0])
        vel = np.array([0.0,0.0,0.0])
        head = 0.0
        head_d = 0.0
        accel = np.array([0.0,0.0,0.0])

    # x only direction
    #   pos = np.array([a/2*np.sin(2*c*time),0.0,-h+b*np.sin(c*time)])
    #   vel = np.array([a*c*np.cos(2*c*time),0.0,b*c*np.cos(c*time)])
    #   head = 0.0
    #   head_d = 0.0
    #   accel = np.array([-2*c**2*a*np.sin(2*c*time),0.0,-b*c**2*np.sin(c*time)])

    return np.array([pos, head, vel, head_d, accel])
