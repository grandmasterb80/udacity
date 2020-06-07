# Write a function 'kalman_filter' that implements a multi-
# dimensional Kalman Filter for the example given

from math import *


class matrix:
    
    # implements basic operations of a matrix class
    
    def __init__(self, value):
        self.value = value
        self.dimx = len(value)
        self.dimy = len(value[0])
        if value == [[]]:
            self.dimx = 0
    
    def zero(self, dimx, dimy):
        # check if valid dimensions
        if dimx < 1 or dimy < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx = dimx
            self.dimy = dimy
            self.value = [[0 for row in range(dimy)] for col in range(dimx)]
    
    def identity(self, dim):
        # check if valid dimension
        if dim < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx = dim
            self.dimy = dim
            self.value = [[0 for row in range(dim)] for col in range(dim)]
            for i in range(dim):
                self.value[i][i] = 1
    
    def show(self):
        for i in range(self.dimx):
            print(self.value[i])
        print(' ')
    
    def __add__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError, "Matrices must be of equal dimensions to add"
        else:
            # add if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] + other.value[i][j]
            return res
    
    def __sub__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError, "Matrices must be of equal dimensions to subtract"
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] - other.value[i][j]
            return res
    
    def __mul__(self, other):
        # check if correct dimensions
        if self.dimy != other.dimx:
            raise ValueError, "Matrices must be m*n and n*p to multiply"
        else:
            # multiply if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, other.dimy)
            for i in range(self.dimx):
                for j in range(other.dimy):
                    for k in range(self.dimy):
                        res.value[i][j] += self.value[i][k] * other.value[k][j]
            return res
    
    def transpose(self):
        # compute transpose
        res = matrix([[]])
        res.zero(self.dimy, self.dimx)
        for i in range(self.dimx):
            for j in range(self.dimy):
                res.value[j][i] = self.value[i][j]
        return res
    
    # Thanks to Ernesto P. Adorio for use of Cholesky and CholeskyInverse functions
    
    def Cholesky(self, ztol=1.0e-5):
        # Computes the upper triangular Cholesky factorization of
        # a positive definite matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)
        
        for i in range(self.dimx):
            S = sum([(res.value[k][i])**2 for k in range(i)])
            d = self.value[i][i] - S
            if abs(d) < ztol:
                res.value[i][i] = 0.0
            else:
                if d < 0.0:
                    raise ValueError, "Matrix not positive-definite"
                res.value[i][i] = sqrt(d)
            for j in range(i+1, self.dimx):
                S = sum([res.value[k][i] * res.value[k][j] for k in range(self.dimx)])
                if abs(S) < ztol:
                    S = 0.0
                try:
                   res.value[i][j] = (self.value[i][j] - S)/res.value[i][i]
                except:
                   raise ValueError, "Zero diagonal"
        return res
    
    def CholeskyInverse(self):
        # Computes inverse of matrix given its Cholesky upper Triangular
        # decomposition of matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)
        
        # Backward step for inverse.
        for j in reversed(range(self.dimx)):
            tjj = self.value[j][j]
            S = sum([self.value[j][k]*res.value[j][k] for k in range(j+1, self.dimx)])
            res.value[j][j] = 1.0/tjj**2 - S/tjj
            for i in reversed(range(j)):
                res.value[j][i] = res.value[i][j] = -sum([self.value[i][k]*res.value[k][j] for k in range(i+1, self.dimx)])/self.value[i][i]
        return res
    
    def inverse(self):
        aux = self.Cholesky()
        res = aux.CholeskyInverse()
        return res
    
    def __repr__(self):
        return repr(self.value)


########################################

# Implement the filter function below
class KalmanFilterClass:
    def __init__(self):
        #self.prev_time = 0

        # Initial State
        self.x = matrix([[0.],
                         [0.]])

        # Uncertainty Matrix
        self.P = matrix([[1000.0, 0.],
                         [0., 1000.0]])

        # Next State Function
        self.F = matrix([[1., 1.],
                         [0., 1.]])

        # Measurement Function
        self.H = matrix([[1., 0.]])

        # Measurement Uncertainty
        self.R = matrix([[0.01]])

        # Identity Matrix
        self.I = matrix([[1., 0.],
                         [0., 1.]])

    #def predict(self,t):
    def predict(self):
        # Calculate dt.
        dt = 1    #t - self.prev_time
        # Put dt into the state transition matrix.
        self.F.value[0][1] = dt
        self.x = self.F * self.x
        self.P = self.F * self.P * self.F.transpose()
        return  #self.x[0][0]


    #def measure_and_update(self,measurements,t):
    def measure_and_update(self,measurements):
        dt = 1    #t - self.prev_time
        #self.F[0,1] = dt
        Z = matrix( [[ measurements ]] )
        y = Z.transpose() - self.H * self.x

        # goal: if deviation of predicted value and the measured value is really high, we increase the Uncertainty matrix
        # abs(y) ==> deviation of measured value to predicted value
        # Subtract the threshold, i.e. all derivation below threshold is considered as measurement noise.
        # Clip and scale it the result to map it to Uncertainty.
        #self.P += np.diag( np.matrix.clip( np.abs(y) - self.threshold, 0.0, 10.0 ) * 1200.0 )

        S = self.H * self.P * self.H.transpose() + self.R
        K = self.P * self.H.transpose() * S.inverse()
        self.x = self.x + K * y
        self.P = (self.I - K * self.H) * self.P

        #self.prev_time = t
        return

    def kfilter(self):
        for n in range(len(measurements)):
            # measurement update
            self.measure_and_update(measurements[n])
            # prediction
            self.predict()
        return self.x, self.P


def kalman_filter(x, P):
    kalman_filter_class = KalmanFilterClass()

    kalman_filter_class.x = x
    kalman_filter_class.P = P
    kalman_filter_class.u = u
    kalman_filter_class.F = F
    kalman_filter_class.H = H
    kalman_filter_class.R = R
    kalman_filter_class.I = I

    return kalman_filter_class.kfilter()

############################################
### use the code below to test your filter!
############################################


measurements = [1, 2, 3]

x = matrix([[0.], [0.]]) # initial state (location and velocity)
P = matrix([[1000., 0.], [0., 1000.]]) # initial uncertainty
u = matrix([[0.], [0.]]) # external motion
F = matrix([[1., 1.], [0, 1.]]) # next state function
H = matrix([[1., 0.]]) # measurement function
R = matrix([[1.]]) # measurement uncertainty
I = matrix([[1., 0.], [0., 1.]]) # identity matrix

print(kalman_filter(x, P))
#[x,P] = kalman_filter.kfilter()
#print "x:", x
#print "P:", P


# output should be:
# x: [[3.9996664447958645], [0.9999998335552873]]
# P: [[2.3318904241194827, 0.9991676099921091], [0.9991676099921067, 0.49950058263974184]]
