from vpython import *
import time

m = 2
Fg = vector(0, -9.8 * 50, 0)
k = 10000000
R = 2
L = 5
L1 = 8 ** (0.5)
L2 = 12 ** (0.5)
g = 9.8 * 10
scene = canvas(x=0, y=0, width=800, height=1200, range=10)
b = 0.0


def SpringForce(r, L):
    return - k * (mag(r) - L) * r / mag(r)


x = arrow(pos=vector(0, 0, 0), axis=vector(1, 0, 0), color=color.green)
y = arrow(pos=vector(0, 0, 0), axis=vector(0, 1, 0), color=color.red)
z = arrow(pos=vector(0, 0, 0), axis=vector(0, 0, 1), color=color.blue)
axis_vector = arrow(pos=vector(0, 0, 0), color=color.yellow, shaftwidth=0.3)
cone(pos=vector(0, -4, 0), axis=vector(0, 4, 0), radius=3, color=color.green)
ball1 = sphere(pos=vector(R * cos(120 * 0 * pi / 180) * cos(-30 * pi / 180) + 2 * -sin(-30 * pi / 180),
                          R * cos(120 * 0 * pi / 180) * sin(-30 * pi / 180) + 2 * cos(-30 * pi / 180),
                          R * sin(120 * 0 * pi / 180))
               , v=vector(0, 0, -80), radius=0.2, color=color.green)
ball2 = sphere(pos=vector(R * cos(120 * 1 * pi / 180) * cos(-30 * pi / 180) + 2 * -sin(-30 * pi / 180),
                          R * cos(120 * 1 * pi / 180) * sin(-30 * pi / 180) + 2 * cos(-30 * pi / 180),
                          R * sin(120 * 1 * pi / 180))
               , v=vector(60, -34.6, 40), radius=0.2, color=color.blue)
ball3 = sphere(pos=vector(R * cos(120 * 2 * pi / 180) * cos(-30 * pi / 180) + 2 * -sin(-30 * pi / 180),
                          R * cos(120 * 2 * pi / 180) * sin(-30 * pi / 180) + 2 * cos(-30 * pi / 180),
                          R * sin(120 * 2 * pi / 180))
               , v=vector(-60, 34.6, 40), radius=0.2, color=color.red)
ballcm = sphere(pos=(ball1.pos + ball2.pos + ball3.pos) / 3 * 2.5,
                color=color.yellow, radius=0.02, make_trail=True, interval=1, retain=45000)

spring_1 = cylinder(radius=0.1)
spring_2 = cylinder(radius=0.1)
spring_3 = cylinder(radius=0.1)
spring_12 = cylinder(radius=0.1)
spring_23 = cylinder(radius=0.1)
spring_31 = cylinder(radius=0.1)

spring_1.pos = vector(0, 0, 0)
spring_2.pos = vector(0, 0, 0)
spring_3.pos = vector(0, 0, 0)
spring_12.pos = vector(ball1.pos)
spring_23.pos = vector(ball2.pos)
spring_31.pos = vector(ball3.pos)

t = 0
dt = 0.0001
while True:
    rate(1 / dt * 0.2)
    t += dt

    spring_12.pos = ball1.pos
    spring_23.pos = ball2.pos
    spring_31.pos = ball3.pos

    spring_1.axis = vector(ball1.pos)
    spring_2.axis = vector(ball2.pos)
    spring_3.axis = vector(ball3.pos)
    spring_12.axis = ball2.pos - ball1.pos
    spring_23.axis = ball3.pos - ball2.pos
    spring_31.axis = ball1.pos - ball3.pos

    F1 = vector(0, -m * g, 0) + SpringForce(spring_1.axis, L1) - SpringForce(spring_12.axis, L2) + SpringForce(
        spring_31.axis, L2)
    F2 = vector(0, -m * g, 0) + SpringForce(spring_2.axis, L1) + SpringForce(spring_12.axis, L2) - SpringForce(
        spring_23.axis, L2)
    F3 = vector(0, -m * g, 0) + SpringForce(spring_3.axis, L1) - SpringForce(spring_31.axis, L2) + SpringForce(
        spring_23.axis, L2)

    ball1.v += (F1 / m) * dt - b * ball1.v * dt / m
    ball1.pos += ball1.v * dt

    ball2.v += (F2 / m) * dt - b * ball2.v * dt / m
    ball2.pos += ball2.v * dt

    ball3.v += (F3 / m) * dt - b * ball3.v * dt / m
    ball3.pos += ball3.v * dt

    print(ball1.pos)
    print(ball2.pos)
    print(ball3.pos)
    print()
    axis_vector.axis = ballcm.pos = (ball1.pos + ball2.pos + ball3.pos) / 3 * 2.5
