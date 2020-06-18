from vpython import *
import time

###基本參數設定
m = 2 #ball mass
Fg = vector(0, -9.8 * 10 * 2, 0) #gravity vector
k = 10000000 #spring coefficient
R = 2 #radius between center of gyro and ball
g = 9.8 * 10
scene = canvas(x=0, y=0, width=800, height=1200, range=10)
b = 0.0 #air resistance coefficient
initial_angle = 30
init_speed = 50
ballnum = 5
theta = 360 / ballnum
L1 = 0 #to be decided
L2 = 0
damp = 10000 #spring damp

### 函式定義
def springForce(r, L):
    return -k * (mag(r) - L) * norm(r)

def SpringDamp(v, r):  #避震器
    cos_theta = dot(v,r)/(mag(v)*mag(r))                    #用向量內積找v和r夾角的餘弦函數
    r_unit_vector = norm(r)                                 #沿彈簧軸方向的單位向量
    projection_vector = mag(v) * cos_theta * r_unit_vector  #計算v在r方向的分量
    spring_damp = - damp * projection_vector                #沿彈簧軸方向的阻力
    return spring_damp

def air(v):
    return -b * v

def getavgpos():
    avgpos = vector(0, 0, 0)
    for i in balls:
        avgpos += i.obj.pos
    avgpos /= ballnum
    return avgpos

def getforce(center, push, pull):
    #print(mag(pull), pull)
    return Fg + springForce(center, L1) - springForce(pull, L2) + springForce(push, L2)
    #return Fg - springForce(pull, L2) + springForce(push, L2)

def dist(p1, p2):
    return sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)

# 物件定義：外圍的球（質點）、彈簧
class ball(object):
    def __init__(self, k, v, col):
        self.obj = sphere(pos=vector(R * cos(theta * k * pi / 180) * cos(-initial_angle * pi / 180) + R * -sin(-initial_angle * pi / 180),
                                     R * cos(theta * k * pi / 180) * sin(-initial_angle * pi / 180) + R * cos(-initial_angle * pi / 180),
                                     R * sin(theta * k * pi / 180)), radius=0.2)
        self.obj.v = v
        self.obj.color = col


class spring():
    def __init__(self, pos1, pos2, col):
        self.obj = cylinder(radius=0.1, pos=pos1, axis=pos2 - pos1, color=col)

    def update(self, p1, p2):
        self.obj.pos = p1
        self.obj.axis = p2 - p1

### 物件初始化
axis_vector = arrow(pos=vector(0, 0, 0), color=color.yellow, shaftwidth=0.3)

# balls = [ball(0, vector(0, 0, 0), color.green), ball(1, vector(0, 0, 0), color.blue),
#        ball(2, vector(0, 0, 0), color.red)]
balls = []
for i in range(ballnum):
    balls.append(ball(i, vector(0, 0, 0), color.white))
ballcm = sphere(pos=vector(0, 0, 0), color=color.yellow, radius=0.02, make_trail=True, interval=1, retain=45000)
L1 = dist(balls[0].obj.pos, vector(0,0,0))
L2 = dist(balls[0].obj.pos, balls[1].obj.pos)

center_spring = []
for i in balls:
    ballcm.pos += i.obj.pos
    center_spring.append(spring(vector(0, 0, 0), i.obj.pos, color.red))
ballcm.pos = ballcm.pos / len(balls) * 2.5

connect_spring = []
colorlist = [color.green, color.blue, color.red, color.white, color.cyan]
for i in range(len(balls) - 1):
    connect_spring.append(spring(balls[i].obj.pos, balls[i + 1].obj.pos, color.white))
connect_spring.append(spring(balls[- 1].obj.pos, balls[0].obj.pos, color.white))
forces = [vector(0, 0, 0)] * len(balls)

# forcearr = []
# for i in range(len(balls)):
#     a = []
#     for j in range(5):
#         a.append(arrow(pos=vector(0, 0, 0), shaftwidth=0.15, color=colorlist[j]))
#     forcearr.append(a)

avgpos = getavgpos()
for i in range(len(balls)):
    rv1 = avgpos - balls[i].obj.pos
    rv2 = balls[i - 1].obj.pos - balls[i].obj.pos
    A = cross(rv1, rv2)
    Vdir = cross(rv1, A)
    balls[i].obj.v = norm(Vdir) * init_speed

t = 0
dt = 0.0001

crossdist = dist(balls[0].obj.pos, balls[- int(ballnum / 2)].obj.pos)
### 圖表、數據分析區
graph_1 = graph(title="Precession angle and time", width=600, height=800, align="bottom")
f1 = gcurve(color=color.blue)
cur_angle = 0
eps = 0.05
t1 = 0

### 執行迴圈
while True:
    #time.sleep(0.01)
    rate(1 / dt * 0.2)
    t += dt

    for i in range(len(balls)):
        center_spring[i].update(vector(0, 0, 0), balls[i].obj.pos)
    for i in range(len(balls) - 1):
        connect_spring[i].update(balls[i].obj.pos, balls[i + 1].obj.pos)
    connect_spring[-1].update(balls[-1].obj.pos, balls[0].obj.pos)

    if ballnum % 2 == 0:
        for i in range(0, len(balls)):
            forces[i] = (getforce(center_spring[i].obj.axis, connect_spring[i - 1].obj.axis, connect_spring[i].obj.axis) \
                         + SpringDamp(balls[i].obj.v, balls[i].obj.pos) + springForce(balls[i].obj.pos - balls[i - int(ballnum / 2)].obj.pos, crossdist))
    else:
        for i in range(0, len(balls)):
            forces[i] = (getforce(center_spring[i].obj.axis, connect_spring[i - 1].obj.axis, connect_spring[i].obj.axis) \
                         + SpringDamp(balls[i].obj.v, balls[i].obj.pos) + springForce(balls[i].obj.pos - balls[i - int(ballnum / 2)].obj.pos, crossdist)) + springForce(balls[i].obj.pos - balls[i - int(ballnum / 2) - 1].obj.pos, crossdist)


    vect = vector(0, 0, 0)
    for i in range(len(balls)):
        vect += balls[i].obj.pos
        balls[i].obj.v += dt * (forces[i] + air(balls[i].obj.v)) / m
        balls[i].obj.pos += balls[i].obj.v * dt
        #print(balls[i].obj.pos.y)

    axis_vector.axis = ballcm.pos = vect / len(balls) * 2.5

    cur_angle = acos(norm(vector(ballcm.pos.x, 0, ballcm.pos.z)).x) / pi * 180
    f1.plot(pos=(t, cur_angle))
    if cur_angle < eps and t - t1 > 0.5:
        print("Precession Cycle is " + str(t - t1) + " seconds")
        t1 = t

