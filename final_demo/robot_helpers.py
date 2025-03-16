# robot_helpers.py

#!/usr/bin/env python3

# Helper functions that control the mechanics of the robot arm

import numpy as np 
import math

class Spline():
    def __init__(self, t0, p0, v0, segment):
        self.t0 = t0
        self.segment = segment
        self.T = segment.time
        p0 = np.array(p0)
        v0 = np.array(v0)
        pf = np.array(segment.joint_angles)
        vf = np.array(segment.joint_vels)

        # Quintic polynomial coefficients
        self.a = p0
        self.b = v0
        self.c = np.zeros_like(p0)
        
        self.d = (10*(pf - p0)/self.T**3 - 6*v0/self.T**2 - 4*vf/self.T**2)
        self.e = (-15*(pf - p0)/self.T**4 + 8*v0/self.T**3 + 7*vf/self.T**3)
        self.f = (6*(pf - p0)/self.T**5 - 3*v0/self.T**4 - 3*vf/self.T**4)

    def evaluate(self, t):
        dt = t - self.t0
        if dt < 0.0:
            dt = 0.0
        p = (self.a + self.b*dt + self.c*dt**2 + self.d*dt**3 + self.e*dt**4 + self.f*dt**5)
        v = (self.b + 2*self.c*dt + 3*self.d*dt**2 + 4*self.e*dt**3 + 5*self.f*dt**4)
        return p.tolist(), v.tolist()
    
    def get_id(self):
        return self.segment.id


def ik_solve(chain, x_goal):
    """
        x_goal: [x, y, z] in meters
        Returns [q1, q2, q3, q4, q5, q6] or None
    """
    initial_guess = [0.0, -3*math.pi/4, -3*math.pi/4, math.pi/2, 0.0, 0.0] # Waiting position
    p = np.array(initial_guess)

    max_iters = 100
    damping = 0.1
    for _ in range(max_iters):
        fkin_pos, _, J_pos, _ = chain.fkin(p.tolist())
        fkin_pos = np.array(fkin_pos)
        err = x_goal - fkin_pos
        norm_err = np.linalg.norm(err)
        if norm_err < 1e-5:
            return p.tolist()

        # Damped-least-squares step
        JT = J_pos.T
        dp = JT @ np.linalg.inv(J_pos @ JT + (damping**2)*np.eye(3)) @ err
        p = p + dp

        # TODO: Fix
        p[0] = np.clip(p[0], -1.20, 1.20)
        p[1] = np.clip(p[1], -2.36, 0.0)
        p[2] = np.clip(p[2], -2.80, 0.0)

        # wrist always points downward
        p[3] = -p[1] + p[2] + math.pi/2

    return None

def interpolate_shoulder_coeffs(elbow_angle):
    e_neg90 = -math.pi/2
    e_zero  =  0.0
    e_pos90 =  math.pi/2

    A_neg90, B_neg90 = (5.0,  -9.0)
    A_zero,  B_zero  = (2.0, -14.0)
    A_pos90, B_pos90 = (-2.6, -9.8)

    if elbow_angle <= e_neg90:
        return A_neg90, B_neg90
    elif elbow_angle >= e_pos90:
        return A_pos90, B_pos90
    elif elbow_angle < e_zero:
        frac = (elbow_angle - e_neg90) / (e_zero - e_neg90)
        A = A_neg90 + frac * (A_zero - A_neg90)
        B = B_neg90 + frac * (B_zero - B_neg90)
        return A, B
    else:
        frac = (elbow_angle - e_zero) / (e_pos90 - e_zero)
        A = A_zero + frac * (A_pos90 - A_zero)
        B = B_zero + frac * (B_pos90 - B_zero)
        return A, B

def gravity_comp(pos):
    """
        Calculate the gravity compensation for the robot given its position
    """
    if len(pos) < 3:
        return [0.0]*6

    theta_shoulder = pos[1]
    theta_elbow    = pos[2]

    A_shoulder, B_shoulder = interpolate_shoulder_coeffs(theta_elbow)
    tau_shoulder = (A_shoulder * math.sin(theta_shoulder)
                    + B_shoulder * math.cos(theta_shoulder))

    A_elbow = 0.8
    B_elbow =  4.2
    tau_elbow = (A_elbow * math.sin(-theta_shoulder + theta_elbow)
                    + B_elbow * math.cos(-theta_shoulder + theta_elbow))

    return [0.0, tau_shoulder, tau_elbow, 0.0, 0.0, 0.0]