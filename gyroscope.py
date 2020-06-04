from vpython import *
import time
m = 2
Fg = vector(0, -9.8, 0)
k = 10000000
R = 2
L1 = sqrt(2*R**2)
L2 = sqrt(3*R**2)
g = 9.8 * 10
scene = canvas(x=0, y=0, width=800, height=1200, range=10)
b = 0.0

def springForce(r,L):
    return -k*(mag(r) - L) * r / mag(r)
def air(v):
    return -b*v

class ball(object):
    def __init__(self,k,v, col):
        self.obj = sphere(pos=vector(R * cos(120 * k * pi / 180) * cos(-30 * pi / 180) + R * -sin(-30 * pi / 180),
                          R * cos(120 * k * pi / 180) * sin(-30 * pi / 180) + R * cos(-30 * pi / 180),
                          R * sin(120 * k * pi / 180)), radius=0.2)
        self.obj.v = v
        self.color = col

class spring():
    def __init__(self, pos1, pos2, col):
        self.obj = arrow(radius=0.1, pos=pos1, axis = pos2 - pos1, color=col)
    def update(self, p1, p2):
        self.obj.pos = p1
        self.obj.axis = p2 - p1

def getforce(center, push, pull):
    global L1, L2, Fg
    return Fg + springForce(center, L1) - springForce(pull, L2) + springForce(push, L2)

axis_vector = arrow(pos=vector(0, 0, 0), color=color.yellow, shaftwidth=0.3)

balls = [ball(0, vector(0, 0, 80), color.green), ball(1, vector(-60, 34.6, 40), color.blue), ball(2, vector(60, -34.6, 40), color.red)]
ballcm = sphere(pos=vector(0, 0, 0),color=color.yellow, radius=0.02, make_trail=True, interval=1, retain=45000)

center_spring = []
for i in balls:
    ballcm.pos += i.obj.pos
    center_spring.append(spring(vector(0, 0, 0), i.obj.pos, color.red))
ballcm.pos = ballcm.pos / len(balls) * 2.5

connect_spring = []
for i in range(len(balls) - 1):
    connect_spring.append(spring(balls[i].obj.pos, balls[i + 1].obj.pos, color.blue))
connect_spring.append(spring(balls[- 1].obj.pos, balls[0].obj.pos, color.blue))

forces = [vector(0, 0, 0)] * len(balls)

t = 0
dt = 0.001
while True:
    time.sleep(1)
    rate(1 / dt)
    t += dt

    for i in range(len(balls)):
        center_spring[i].update(vector(0, 0, 0), balls[i].obj.pos)
    for i in range(len(balls) - 1):
        connect_spring[i].update(balls[i].obj.pos, balls[i + 1].obj.pos)
    connect_spring[len(balls) - 1].update(balls[len(balls) - 1].obj.pos, balls[0].obj.pos)

    # for i in range(0, len(balls)):
    #     forces[i] = getforce(center_spring[i].obj.axis, connect_spring[i - 1].obj.axis, connect_spring[i].obj.axis)
    forces[0] = vector(0, -m * g, 0) + springForce(center_spring[0].obj.axis, L1) - springForce(connect_spring[0].obj.axis, L2) + springForce(
        connect_spring[2].obj.axis, L2)
    forces[1] = vector(0, -m * g, 0) + springForce(center_spring[1].obj.axis, L1) - springForce(connect_spring[1].obj.axis, L2) + springForce(
        connect_spring[0].obj.axis, L2)
    forces[2] = vector(0, -m * g, 0) + springForce(center_spring[2].obj.axis, L1) - springForce(connect_spring[2].obj.axis, L2) + springForce(
        connect_spring[1].obj.axis, L2)

    vect = vector(0, 0, 0)
    for i in range(len(balls)):
        vect += balls[i].obj.pos
        balls[i].obj.v += dt * (forces[i] + air(balls[i].obj.v)) / m
        balls[i].obj.pos += balls[i].obj.v * dt
        
    axis_vector.axis = ballcm.pos = vect / len(balls) * 2.5

