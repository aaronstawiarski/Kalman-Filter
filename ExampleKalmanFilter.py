import numpy as np
# offsets of each variable in the state vector
iX = 0
iV = 1
iZ = 2
NUMVARS = iV + 1



class KF:
    def __init__(self, initial_x : float,initial_v: float, accel_variance: float) -> None: #declaring the types of each variable
        # mean of state GRV
        self.x = np.zeros(NUMVARS)
        self.x[iX]=initial_x
        self.x[iV] = initial_v  # sets the objects x as an array of pos and vel
        self.accel_variance = accel_variance # sets the objects accel variance as the input accel variance
        #covariance fo state GRV
        self.P = np.eye(NUMVARS) # sets the objects P matrix as a 2x2 identity matrix

    def predict(self, dt: float) -> None: # defines predict function, w/ dt being float and the function having no return type
        # x = F * x 
        # P = F P Ft+ G Gt a
        F = np.eye(NUMVARS)
        F[iX,iV] = dt  # sets F matrix as 2x2 array based on KF equations
        new_x = F.dot(self.x) # makes the new x array as F mult. w/ old x
        G = np.zeros((2,1))

        G[iX] = 0.5 * dt **2 
        G[iV] = dt # creates matrix G as a 2 x 1 vector 
        new_P = F.dot(self.P).dot(F.T) + G.dot(G.T) * self.accel_variance # creates new P matrix as F times old P times F transposed times G times G transposed

        # now, setting self properties to the newly calculated 

        self.P = new_P #assigns old P to new P for iterative purposes
        self.x = new_x # assigns old X to new X for iterative purposes

    def update(self, meas_value: float , meas_variance: float) -> None:
        # y = z- H x
        # S= H P Htkj +R 
        # K = P Ht S^-1
        # x = x + K y
        # P = (I - K H )* P
        z= np.array([meas_value])
        R = np.array([meas_variance])

        H = np.array([1,0]).reshape((1,2))


        y = z - H.dot(self.x)
        S =H.dot(self.P).dot(H.T) + R
        K = self.P.dot(H.T).dot(np.linalg.inv(S))

        new_x = self.x + K.dot(y)
        new_P = (np.eye(2) - K.dot(H)).dot(self.P)

        self.P = new_P
        self.x = new_x 
  
    @property 
    def cov(self) -> np.array: # defines covariance function, taking object as input and outputting type array
        return self.P #returns P as the ' covariance matrix '
    
    @property
    def mean(self) -> np.array: # defines mean function, taking object as input and outputting type array
        return self.x # returns the state array as output
    @property
    def pos(self) -> float: # defines pos function, outputting type float
        return self.x[iX] # returns element 0( or 1)  of the state function, which is postion

    @property
    def vel(self) -> float: #defines vel function, returns type float
        return self.x[iV] # returns idx 1 (pos 2) of the state function, which is velocity

