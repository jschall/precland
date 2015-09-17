from pymavlink.rotmat import Vector3
from math import *
from visual import *

def sqrt_ctrl(error, p, alim):
    if p == 0 or alim == 0:
        return error*p

    linear_dist = alim/p**2

    if error > linear_dist:
        return sqrt(2*alim*(error-(linear_dist/2)))
    elif error < -linear_dist:
        return -sqrt(2*alim*(-error-(linear_dist/2)))
    else:
        return error*p

def constrain(x,l,h):
    if x > h:
        return h
    if x < l:
        return l
    return x

class Copter:
    def __init__(self):
        self.pos = Vector3(0.,10.,-20.)
        self.vel = Vector3(0.,0.,0.)
        self.vert_accel = 0.0
        self.roll_ang = radians(45.0)
        self.roll_ang_vel = 0.0
        self.roll_ang_acc = 0.0
        self.roll_ang_acc_in = 0.0
        self.roll_ang_acc_lim = 2.0
        self.drag_coeff = 9.81/18
        self.wind = 5
        self.fov = radians(16)
        self.last_roll_ang_vel_err = 0

        self.vel_target = Vector3(0,0,0)

        self.vel_integrator = 0.0

        self.tar_pos = Vector3(0,0,0)

        cone_axis = -Vector3(0,-sin(self.roll_ang),cos(self.roll_ang))*30

        self.copter_vis = cone(pos=(self.pos-cone_axis).get_vpy_vec(), axis=cone_axis.get_vpy_vec(), opacity=.1, radius=cone_axis.length()*tan(self.fov/2))
        self.target_vis = sphere(pos=self.tar_pos.get_vpy_vec(),radius=0.5)

    def get_dir_to_target(self):
        dir_to_target = atan2(self.tar_pos.y-self.pos.y,self.tar_pos.z-self.pos.z)
        if abs(self.roll_ang+dir_to_target) > self.fov/2:
            return None
        return dir_to_target

    def update_controller(self,dt):
        self.vert_accel = (self.vel_target.z-self.vel.z)*1.4

        self.vel_integrator += (self.vel_target.y-self.vel.y)*.7*dt
        roll_ang_target = atan(((self.vel_target.y-self.vel.y)*1.4 + self.vel_integrator)/9.81)

        if roll_ang_target > radians(45):
            roll_ang_target = radians(45)
        elif roll_ang_target < -radians(45):
            roll_ang_target = -radians(45)

        roll_ang_error = roll_ang_target - self.roll_ang

        if roll_ang_error > pi:
            roll_ang_error -= 2*pi
        elif roll_ang_error < -pi:
            roll_ang_error += 2*pi

        roll_ang_vel_target = sqrt_ctrl(roll_ang_error, 4, self.roll_ang_acc_lim*.5)
        roll_ang_vel_err = roll_ang_vel_target-self.roll_ang_vel
        self.roll_ang_acc_in = (roll_ang_vel_err)*8.0 + (roll_ang_vel_err-self.last_roll_ang_vel_err)*dt*16
        self.last_roll_ang_vel_err = roll_ang_vel_err

    def update(self,dt):
        if self.roll_ang_acc_in>self.roll_ang_acc_lim:
            self.roll_ang_acc_in=self.roll_ang_acc_lim
        elif self.roll_ang_acc_in<-self.roll_ang_acc_lim:
            self.roll_ang_acc_in=-self.roll_ang_acc_lim

        self.roll_ang_acc += (self.roll_ang_acc_in-self.roll_ang_acc)*dt/(dt+.1)

        self.roll_ang_vel += self.roll_ang_acc*dt*0.5
        self.roll_ang += self.roll_ang_vel*dt
        self.roll_ang_vel += self.roll_ang_acc*dt*0.5

        self.vel.x = 0
        self.vel.y += ((9.81+self.vert_accel)*tan(self.roll_ang)-(self.vel.y+self.wind)*self.drag_coeff)*dt*0.5
        self.vel.z += (self.vert_accel-self.vel.z*self.drag_coeff)*dt*0.5

        self.pos.x = 0
        self.pos.y += self.vel.y*dt
        self.pos.z += self.vel.z*dt

        self.vel.x = 0
        self.vel.y += ((9.81+self.vert_accel)*tan(self.roll_ang)-(self.vel.y+self.wind)*self.drag_coeff)*dt*0.5
        self.vel.z += (self.vert_accel-self.vel.z*self.drag_coeff)*dt*0.5

        cone_axis = -Vector3(0,-sin(self.roll_ang),cos(self.roll_ang))*30

        self.copter_vis.axis = cone_axis.get_vpy_vec()
        self.copter_vis.pos  = (self.pos-cone_axis).get_vpy_vec()
        self.target_vis.pos  = self.tar_pos.get_vpy_vec()

c = Copter()

t = 0.0
while True:
    rate(100)
    c.vel_target.y = -2 if (t/5)%2 < 1 else 2
    c.update_controller(0.1)
    c.update(0.1)
    c.get_dir_to_target()
    t += 0.1
