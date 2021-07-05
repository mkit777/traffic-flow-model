import math
from typing import List
import matplotlib.pyplot as plt
import random

#----------------------------------------
# HCCA 模型实现
# author: zhy
#----------------------------------------

# 激进型驾驶员
DRIVER_RADICAL = 0
# 其他类型驾驶员
DRIVER_OTHER   = 1

# 正常运行
STAGE_RUNNING = 0
# 正在变道
STAGE_CHANING = 1


class Car:
    def __init__(self, speed, stage, type):
        '''
        存储车辆元胞状态

        Arguments:
        speed: 速度
        stage: 运行阶段，可取值 STAGE_RUNNING | STAGE_CHANING
        type:  驾驶员类型，可取值 DRIVER_RADICAL | DRIVER_OTHER
        '''
        self.speed = speed
        self.stage = stage
        self.type = type


class HCCA:
    def __init__(self, length, densitys, max_speed,  radical_ratio, a = 1, b = 1):
        '''
        创建STCA模型

        Arguments
        length:        元胞长度
        densitys:      车流密度, list,分别为两个车道的密度
        max_speed:     最大车速
        radical_ratio: 激进驾驶员比例
        p_slow:        随机慢化概率
        a:             减速度
        b:             加速度
        '''
        # 元胞数量
        self.length = length
        # 车辆密度
        self.densitys = densitys
        # 最大速度
        self.max_speed = max_speed
        # 激进驾驶员比例
        self.radical_ratio = radical_ratio
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
        car_num1 = int(self.length * self.densitys[0])
        car_num2 = int(self.length * self.densitys[1])
        radical_num = int((car_num1 + car_num1) * self.radical_ratio)
        
        radicals = [Car(random.randint(1, self.max_speed), STAGE_RUNNING, DRIVER_RADICAL) for _ in range(radical_num)]
        ohters = [Car(random.randint(1, self.max_speed), STAGE_RUNNING, DRIVER_OTHER) for _ in range(car_num1 + car_num2 - radical_num)]
        all = [*radicals, *ohters]
        random.shuffle(all)

        cells = [
            [*all[:car_num1], *[None for _ in range(self.length - car_num1)]],
            [*all[car_num1:], *[None for _ in range(self.length - car_num2)]]
        ]

        for i in range(2):
            random.shuffle(cells[i])

        return cells

    def update(self) -> List[float]:
        '''
        更新元胞状态
        '''
        nexts = [[None] * self.length, [None] * self.length]

        # 遍历车道
        for lidx, lane in enumerate(self.cells):
            for  cidx, car in enumerate(lane):
                if car is None:
                    continue

                fidx, front = self._find_front(cidx, lidx)
                # 最小安全车速计算
                vp = min(min(self.max_speed - 1, front.speed), max(0, self._calc_dn(fidx, lidx)))

                # 变道，只对正常行驶的车辆进行判断，处在变道期的车辆不进行判断       
                if car.stage == STAGE_RUNNING:
                    # 换道动机
                    if car.type == DRIVER_RADICAL:
                        s1 = self._calc_dn(cidx, lidx) < min(car.speed + 1, self.max_speed) + vp
                        s2 = self._calc_dn_other(cidx, lidx) > min(car.speed + 1, self.max_speed) + vp
                    else:
                        s1 = self._calc_dn(cidx, lidx) < min(car.speed + 1, self.max_speed)
                        s2 = self._calc_dn_other(cidx, lidx) > min(car.speed + 1, self.max_speed)

                    # 安全条件
                    s3 = self._calc_dn_back(cidx, lidx) >= self.max_speed
                    # 前景理论
                    evn = self._calc_evn(car, cidx, lidx)
                    s4 = evn[0] > evn[1]
                    if s1 and s2 and s3 and s4:
                        car.stage = STAGE_CHANING
                        nexts[(lidx + 1) % 2][cidx] = car
                        continue
                
                car.stage = STAGE_RUNNING
                # 随机慢化概率
                pn = self._calc_pn(car, cidx, lidx)
                # 加速
                car.speed = min(car.speed + self.a, self.max_speed)
                # 安全防护
                if car.type == DRIVER_RADICAL:
                    car.speed = min(car.speed, self._calc_dn(cidx, lidx) + vp)
                else:
                    car.speed = min(car.speed, self._calc_dn(cidx, lidx))
                # 随机慢化
                if random.random() <= pn:
                    car.speed = min(car.speed - self.b, 1)
                # 位置更新
                next = cidx + car.speed
                nexts[lidx][next % self.length] = car
        self.cells = nexts
        return self.cells

    def _calc_pn(self, car, current, current_lane):
        # 计算相对速度差影响
        speed_sum = 0
        count = 0
        for c in self.cells[current_lane]:
            if c is None:
                continue
            speed_sum += c.speed
            count += 1
        lbd1 = count / max(speed_sum, 0.001)
        fidx, front_car = self._find_front(current, current_lane)
        f_speed = front_car.speed if front_car is not None else car.speed
        f = lbd1 * math.e ** (- lbd1 * ( car.speed - f_speed ))

        # 计算安全车距影响
        dn_sum = 0
        count = 0
        for cidx, car in enumerate(self.cells[current_lane]):
            if car is None:
                continue
            dn_sum += self._calc_dn(cidx, current_lane)
            count += 1
        lbd2 = count / dn_sum
        g = lbd2 * math.e ** (- lbd2 * (self._calc_dn(current, current_lane) - self.max_speed))
        
        return f * g

    def _calc_evn(self, car, current, current_lane):
        alpha = 0.89
        beta = 0.92
        lbd = 2.25

        p_jam = 0.5
        chi = 0.61
        delta = 0.69

        dn = self._calc_dn(current, current_lane)

        # 心理预期
        k = dn / self.max_speed
        # 换道后行驶时间
        t1 = self._calc_dn_other(current, current_lane) / max(min(car.speed + self.a, self.max_speed), 0.001)
        # 换道收益
        y1 = k - t1
        # 换道价值
        v1 = t1 **  alpha if y1 >= 0 else - lbd * (-y1) ** beta

        # 不换道行驶时间
        t2 = dn / max(car.speed, 0.001)
        # 不换道收益
        y2 = k - t2
        # 不换道价值
        v2 = t2 **  alpha if y2 >= 0 else - lbd * (-y2) ** beta

        # 拥堵概率感知
        wp = self._func_jam_feeling(chi, p_jam)
        wn = self._func_jam_feeling(delta, p_jam)

        # 前景值计算
        ev1 = wp * v1 + wn * v1
        ev2 = wp * v2 + wn * v2
        return ev1, ev2 

    def _func_jam_feeling(self, delta, p):
        '''
        拥堵感知概率
        '''
        return p ** delta / (p ** delta + (1-p)**delta) ** (1/delta)

    def _find_front(self, current, current_lane):
        '''
        查找指定车的前车
        '''
        cursor = current + 1
        while True:
            if cursor == self.length:
                cursor = 0
            car = self.cells[current_lane][cursor]
            if car is None:
                cursor += 1
                continue
            if cursor == current:
                return None, None
            return cursor, car

    def _calc_dn_back(self, current, current_lane):
        '''
        计算与旁道后车的距离
        '''
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

    def _calc_dn_other(self, current, current_lane):
        '''
        计算与旁道前车的距离
        '''
        other_lane =  (current + 1) % 2
        if self.cells[other_lane][current] is not None:
            return 0
        return self._calc_dn(current, other_lane)

    def _calc_dn(self, current, current_lane):
        '''
        计算与前车的距离

        Arguments:
        current:      当前车的位置
        current_lane: 当前车道
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


def plot_his(history, start):
    plt.subplot(121)    
    for hidx, data in enumerate(history[start:]):
        for cidx, c in enumerate(data[0]):
            if c is None:
                continue
            plt.plot(cidx, start + hidx, ',k')
    plt.subplot(122)
    for hidx, data in enumerate(history[start:]):
        for cidx, c in enumerate(data[1]):
            if c is None:
                continue
            plt.plot(cidx, start + hidx, ',k')
    plt.show()


if __name__ == '__main__':
    # 创建模型
    model = HCCA(100, [0.1, 0.2], 5, 0.2)
    history = []
    # 创建绘图器
    ploter = CaPloter()
    # 迭代 1000 次
    for i in range(11000):
        # 更新模型
        cells = model.update()
        history.append([[ c.speed if c is not None else None for c in lane] for lane in cells])
        print(i)
        ploter.plot(i, cells)
        ploter.pause()
    plot_his(history, 10500)

