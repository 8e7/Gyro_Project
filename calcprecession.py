from vpython import *
import time

###基本參數設定
m = 2 #ball mass
Fg = vector(0, -9.8 * 10, 0) #gravity vector
k = 10000000 #spring coefficient
R = 2 #radius between center of gyro and ball
g = 9.8 * 10
scene = canvas(x=0, y=0, width=800, height=1200, range=10)
b = 0.0 #air resistance coefficient
ballnum = 6
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

def getavgpos(balls):
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
    def __init__(self, k, v, col, initial_angle):
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

dt = 0.0002

# ### 圖表、數據分析區
# graph_1 = graph(title="Precession angle and time", width=600, height=800, align="bottom")
# f1 = gcurve(color=color.blue)
# cur_angle = 0
eps = 0.05
# t1 = 0


def calcprecessioncycle(initial_angle, init_speed):
    balls = (ball(0, vector(0, 0, 0), color.white, initial_angle))
    L1 = dist(balls.obj.pos, vector(0, 0, 0))
    r = sqrt(L1 ** 2 - R ** 2)
    omega = init_speed / R
    M = m * ballnum
    I = 0
    for i in range(ballnum):
        I += m * R**2
    p_rate = M * r * g / (I*omega)
    return 2 * R * pi / p_rate


data = []
for i in range(1, 10, 1):
    arr = []
    for j in range(1, 11, 1):
        arr.append(0)
    data.append(arr)
print(data)
for i in range(10, 100, 10):
    for j in range(10, 110, 10):
        data[int(i / 10) - 1][int(j / 10) - 1] = calcprecessioncycle(i, j)
        print(data[int(i / 10) - 1][int(j / 10) - 1], end=", ")
print(data)
