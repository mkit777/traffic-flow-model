from typing import List
import matplotlib.pyplot as plt
import random

#----------------------------------------
# NaSch 模型实现
# author: zhy
#----------------------------------------

class NaSch:
    def __init__(self, length, density, max_speed, p_slow = 0.1, a = 1, b = 1) -> None:
        '''
        创建NaSch模型

        Arguments
        length:     元包长度（个数）
        density:    车流密度
        max_speed:  最大车速
        p_slow:     随机慢化概率
        a:          减速度
        b:          加速度
        '''
        # 元包数量
        self.length = length
        # 车辆密度
        self.density = density
        # 车辆总数
        self.car_num = int(self.length * self.density)
        # 最大速度
        self.max_speed = max_speed
        # 随机慢化率
        self.p_slow = p_slow
        # 加速度
        self.a = a
        # 减速度
        self.b = b
        # 元包信息
        self.cells = self._init_cells()
        
    def _init_cells(self):
        '''
        初始化元包状态
        '''
        cells = [None] * (self.length - self.car_num)
        cars = [random.randint(1, self.max_speed) for _ in range(self.car_num)]
        cells += cars
        random.shuffle(cells)
        return cells

    def update(self) -> List[float]:
        '''
        更新元包状态
        '''
        nexts = [None] * self.length
        for idx, speed in enumerate(self.cells):
            if speed is None:
                continue
            # 加速
            speed = min(speed + self.a, self.max_speed)
            # 安全防护
            speed = min(speed, self._clc_dn(idx))
            # 随机慢化
            if random.random() <= self.p_slow:
                speed = min(speed - self.b, 1)
            # 位置更新
            next = idx + speed
            nexts[next % self.length] = speed
        self.cells = nexts
        return self.cells

    def _clc_dn(self, current):
        '''
        计算与前车的距离

        Arguments:
        current: int 当前车的位置
        '''
        cursor = current + 1
        dn = 0
        while True:
            if cursor == self.length:
                cursor = 0
            speed = self.cells[cursor]
            if speed is not None:
                return dn
            dn += 1
            cursor += 1
            
 

class CaPloter:
    def __init__(self) -> None:
        self.fig = plt.figure()
        self.ax1  = plt.subplot(111)
        self.ax1.set_xticks([])
        self.ax1.set_yticks([])
        self.ax1.set_xticklabels([])
        self.ax1.set_yticklabels([])
        self.ax1.minorticks_off()

    def plot(self, step, cells: List[float]):
        '''
        绘制元包状态

        Arguments
        step:  当前步数
        cells: 元包状态
        '''
        self.ax1.spy([[ 1 if c is not None else 0  for c in cells]])
        self.ax1.set_xlabel(f'step {step}')

    def pause(self):
        plt.pause(0.02)


if __name__ == '__main__':
    # 创建模型
    model = NaSch(100, 0.2, 4, 0.01)
    # 创建绘图器
    ploter = CaPloter()
    # 迭代 1000 次
    for i in range(1000):
        # 更新模型
        cells = model.update()
        ploter.plot(i, cells)
        ploter.pause()
    plt.show()