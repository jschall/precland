import numpy as np
from math import *

class ExtendedKalmanFilter:
    def update(self, meas):
        meas = np.matrix(meas).T
        P = self.covariance
        H = self.H()
        R = self.R()
        I = np.matrix(np.eye(P.shape[0]))
        x_hat = self.state

        y_tilde = meas - self.h()

        S = H * P * H.T + R
        K = P * H.T * S.I

        x_hat = x_hat + K*y_tilde
        P = (I-K*H)*P

        self.state = x_hat
        self.covariance = P

    def predict(self, dt=0, u=None):
        P = self.covariance
        F = self.F(dt)
        Q = self.Q(dt)

        x_hat = self.f(dt, u)
        P = F*P*F.T+Q

        self.state = x_hat
        self.covariance = P

    def h(self, dt):
        return H()*self.state

    def f(self, dt, u):
        B = self.B(dt) if u is not None else None

        x_hat = self.state

        x_hat = F*x_hat
        if B is not None:
            x_hat += B*u

        return x_hat

    def set_params(self, params):
        self.params = params


class TargetEstimator(ExtendedKalmanFilter):
    def __init__(self):
        # state vector: open-circuit voltage, current, battery resistance
        # x hat on wikipedia
        self.state = np.matrix([
            [ 0. ], # Tx
            [ 0. ], # Tz
            [ 0. ], # Px
            [ -10. ]  # Pz
            ])

        # P on wikipedia
        self.covariance = np.matrix([
            [ 16.**2 ,   0   ,   0   ,   0   ],
            [   0   , 4.**2 ,   0   ,   0   ],
            [   0   ,   0   , 0.**2 ,   0   ],
            [   0   ,   0   ,   0   , 0.**2 ]
            ])

    def f(self, dt, u):
        Tx = self.state.item(0)
        Tz = self.state.item(1)
        Px = self.state.item(2)
        Pz = self.state.item(3)

        return np.matrix([
            [ Tx ],
            [ Tz ],
            [ Px ],
            [ Pz ]
            ])


    def F(self, dt):
        return np.matrix(np.eye(self.state.shape[0]))

    def R(self):
        return np.matrix([
            [ .05**2 ,    0   ,       0       ],
            [   0    , .05**2 ,       0       ],
            [   0    ,    0   , radians(0.0001)**2 ]
            ])

    def Q(self,dt):
        Tx_PNOISE = 0.1
        Tz_PNOISE = 0.1
        Px_PNOISE = 0.2
        Pz_PNOISE = 0.2
        return np.matrix([
            [ (Tx_PNOISE*dt)**2 ,         0         ,         0         ,         0         ],
            [         0         , (Tz_PNOISE*dt)**2 ,         0         ,         0         ],
            [         0         ,         0         , (Px_PNOISE*dt)**2 ,         0         ],
            [         0         ,         0         ,         0         , (Pz_PNOISE*dt)**2 ]
            ])

    def h(self):
        Tx = self.state.item(0)
        Tz = self.state.item(1)
        Px = self.state.item(2)
        Pz = self.state.item(3)
        return np.matrix([
            [ Px ],
            [ Pz ],
            [ atan2(Tx-Px,-Pz-(-Tz)) ]
            ])

    def H(self):
        Tx = self.state.item(0)
        Tz = self.state.item(1)
        Px = self.state.item(2)
        Pz = self.state.item(3)

        return np.matrix([
            [ 0 , 0 , 1 , 0 ],
            [ 0 , 0 , 0 , 1 ],
            [ (Tz-Pz)/((Tx-Px)**2+(Tz-Pz)**2) , (Px-Tx)/((Tx-Px)**2+(Tz-Pz)**2) , -(Tz-Pz)/((Tx-Px)**2+(Tz-Pz)**2) , -(Px-Tx)/((Tx-Px)**2+(Tz-Pz)**2) ]
            ])

    def __str__(self):
        return str(self.state)
