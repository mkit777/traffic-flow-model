from typing import List
import matplotlib.pyplot as plt
import random

#----------------------------------------
# STCA 模型实现
# author: zhy
#----------------------------------------


# 正常运行
RUNNING = 0
# 正在变道
CHANING = 1

class Car:
    def __init__(self, speed, stage):
        '''
        存储车辆元胞状态

        Arguments:
        speed: 速度
        stage: 运行阶段，可取值 RUNNING | CHANING
        '''
        self.speed = speed
        self.stage = stage


class STCA:
    def __init__(self, length, density, max_speed, p_slow = 0.1, a = 1, b = 1) -> None:
        '''
        创建STCA模型

        Arguments
        length:     元胞长度（个数）
        density:    车流密度(两个车道的密度)
        max_speed:  最大车速
        p_slow:     随机慢化概率
        a:          减速度
        b:          加速度
        '''
        # 元胞数量
        self.length = length
        # 车辆密度
        self.density = density
        # 车辆总数
        self.car_num = int(self.length * 2 * self.density)
        # 最大速度
        self.max_speed = max_speed
        # 随机慢化率
        self.p_slow = p_slow
        # 加速度
        self.a = a
        # 减速度
        self.b = b
        # 元胞
        self.cells = self._init_cells()
        
    def _init_cells(self):
        '''
        初始化元胞状态
        '''
        cells = [None] * (self.length * 2 - self.car_num)
        cars = [Car(random.randint(1, self.max_speed), RUNNING) for _ in range(self.car_num)]
        cells += cars
        random.shuffle(cells)
        cells = [cells[:self.length], cells[self.length:]]
        return cells

    def update(self) -> List[float]:
        '''
        更新元胞状态
        '''
        nexts = [[None] * self.length, [None] * self.length]

        # 遍历车道
        for lidx, lane in enumerate(self.cells):
            for idx, car in enumerate(lane):
                if car is None:
                    continue
                # 变道，只对正常行驶的车辆进行判断，处在变道期的车辆不进行判断       
                if car.stage == RUNNING:
                    # 换道动机
                    s1 = self._clc_dn(idx, lidx) < min(car.speed + 1, self.max_speed)
                    s2 = self._clc_dn_other(idx, lidx) > min(car.speed + 1, self.max_speed)
                    # 安全条件
                    s3 = self._clc_dn_back(idx, lidx) > self.max_speed
                    if s1 and s2 and s3:
                        car.stage = CHANING
                        nexts[(lidx + 1) % 2][idx] = car
                        continue
                
                car.stage = RUNNING
                # 加速
                car.speed = min(car.speed + self.a, self.max_speed)
                # 安全防护
                car.speed = min(car.speed, self._clc_dn(idx, lidx))
                # 随机慢化
                if random.random() <= self.p_slow:
                    car.speed = min(car.speed - self.b, 1)
                # 位置更新
                next = idx + car.speed
                nexts[lidx][next % self.length] = car
        self.cells = nexts
        return self.cells

    def _clc_dn_back(self, current, current_lane):
        cursor = current - 1
        dn = 0
        while True:
            if cursor == -1:
                cursor = self.length - 1
            speed = self.cells[(current_lane + 1) % 2][cursor]
            if speed is not None:
                return dn
            dn += 1
            # 防止某个车道为空，进入死循环
            if dn >= self.length - 1:
                return self.length - 1
            cursor -= 1

    def _clc_dn_other(self, current, current_lane):
        other_lane =  (current + 1) % 2
        if self.cells[other_lane][current] is not None:
            return 0
        return self._clc_dn(current, other_lane)

    def _clc_dn(self, current, current_lane):
        '''
        计算与前车的距离

        Arguments:
        current:      int 当前车的位置
        current_lane: int 当前车道
        '''
        cursor = current + 1
        dn = 0
        while True:
            if cursor == self.length:
                cursor = 0
            speed = self.cells[current_lane][cursor]
            if speed is not None:
                return dn
            dn += 1
            # 防止某个车道为空，进入死循环
            if dn >= self.length - 1:
                return self.length - 1
            cursor += 1
            
 
class CaPloter:
    def __init__(self) -> None:
        self.fig = plt.figure()
        self.ax1  = plt.subplot(111)

    def plot(self, step, cells: List[List[Car]]):
        '''
        绘制元胞状态

        Arguments
        step:  当前步数
        cells: 元胞状态
        '''
        self.ax1.imshow([[ 1 if c is not None else 0  for c in row]  for row in cells], cmap='binary')
        self.ax1.set_xlabel(f'step {step}')

    def pause(self):
        plt.pause(0.1)


if __name__ == '__main__':
    # 创建模型
    model = STCA(100, 0.1, 4, 0.02)
    # 创建绘图器
    ploter = CaPloter()
    # 迭代 1000 次
    for i in range(1000):
        # 更新模型
        cells = model.update()
        ploter.plot(i, cells)
        ploter.pause()
    plt.show()