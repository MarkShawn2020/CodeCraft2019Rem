#-*-coding:utf-8-*- 
__author__ = 'MarkShawn'

import numpy as np
from queue import PriorityQueue
from data import Data


class Road(object):

    def __init__(self, id, length, speed_limited, lanes_cnt, from_cross_id, to_cross, is_duplex):
        self.id = id
        self.length = length
        self.speed_limited = speed_limited
        self.lanes_cnt = lanes_cnt
        self.from_cross_id = from_cross_id
        self.to_cross_id = to_cross
        self.is_duplex = is_duplex

        # VID元祖是指道路的指向，即数学上的向量，用于链接道路字典
        self.vid = (self.from_cross_id, self.to_cross_id)


        # 使用Numpy数组保存小车的车位信息，数组的横轴是道路的宽度，纵轴是道路的长度，即三行五列的数组表示3个车道长度为5
        self.car_arr = np.empty([self.lanes_cnt, self.length], dtype=object)
        self.pos_arr = np.empty([self.lanes_cnt, self.length], dtype=object)

        self.normal_garage = PriorityQueue()
        self.prior_garage = PriorityQueue()
        self.preset_cars_loaded_cnt = 0

        self.head_cars = set()
        self.cars_load_cnt = 0

    @property
    def payload(self):          # 道路的最大负载量，即车位的总数，长乘以宽
        return self.car_arr.size


    def add_car(self, car):
        self.car_arr[car.lane, car.row] = car
        self.cars_load_cnt += 1
        Data.road_use_arr[self.vid] += 1 / self.payload

    def del_car(self, car):
        self.car_arr[car.lane, car.row] = None
        self.cars_load_cnt -= 1
        Data.road_use_arr[self.vid] -= 1 / self.payload


    def push_head_car(self, car):
        self.head_cars.add(car)

    def pop_head_car(self, car):
        self.head_cars.remove(car)

    def get_min_v(self, lane_seq):
        v_list = [i.speed for i in self.car_arr[lane_seq, :] if i]
        if v_list:
            return min(min(v_list), self.speed_limited)
        return self.speed_limited


    """
    ===========================
    以下方法用于获取道路的头部小车与尾部进入逻辑
    ===========================
    """
    @property
    def is_blocking(self):
        """
        尾部是否满载堵塞，即所有车道入口均被小车堵塞，且处于停止状态
        此函数可用于简化判断该道路能不能进入新的小车
        :return:
        """
        for i in self.car_arr[:, 0]:
            if not i or i.is_waiting:
                return False
        return True

    def get_block_car_at_the_entrance(self, max_dis):
        """
        Has MaxPos: can enter
        Else if:
        BlockCar is waiting: cant enter, delay or waiting
        BlockCar is stopped: cant enter, road is blocking, delay or marked to stop
        """
        blocked_car = None
        max_pos = None
        for lane_seq, lane_cars in enumerate(self.car_arr[:, :max_dis]):
            for row_seq, car in enumerate(lane_cars):
                if not car:
                    max_pos = (lane_seq, row_seq)   # 没车，标记为可以进入的道路点
                elif car.is_waiting:
                    return car, None                # 有车，正在等待吗，则标记为无法进入
                else:
                    blocked_car = car               # 有车，已经停止，继续遍历其他道路点
                    break
            if max_pos:
                return blocked_car, max_pos         # 如有有可以进入的道路点，则直接进入
        return blocked_car, max_pos                 # 最终结果，如果有车且停止，但没路，说明


    def get_first_waiting_car(self):
        if self.head_cars:
            return sorted(self.head_cars, key=lambda x: (-x.priority, -x.row, x.lane))[0]




    """
    ========================
    神奇车库的装载
    ========================
    """
    def add_car_to_garage(self, car):
        """
        装载要出发的小车到道路上，并且标记待完成小车数量加一
        """

        Data.total_loaded += 1
        if car.priority:
            self.prior_garage.put(car)
            Data.prior_loaded += 1
        else:
            self.normal_garage.put(car)

        if car.preset:
            Data.preset_loaded += 1


    """
    ===============================
    以下两个函数分别对应于调度步骤一和步骤二里车队的标记
    ===============================
    """

    def mark_cars_on_this_lane_in_step_1(self, lane_seq):
        """
        所有的车此时应该都是stop的状态，不存在waiting；所以要将car标记成stop或者wait
        """
        block_car = None
        HAS_WAITING_CAR = False
        for car in self.car_arr[lane_seq, ::-1]:
            if car:
                if not block_car:
                    if car.will_exceed_this_road:
                        car.wait()
                        self.push_head_car(car)

                        Data.to_schedule += 1
                        HAS_WAITING_CAR = True
                    else:
                        car.move(row=car.max_row_on_this_road, step_seq=1, desc='Mark')

                elif car.row + car.max_v_on_this_road >= block_car.row:
                    if block_car.is_waiting:
                        car.wait(block_car)

                        Data.to_schedule += 1
                        HAS_WAITING_CAR = True
                    else:
                        car.move(row=block_car.row-1, block_dis=car.max_row_on_this_road-block_car.row+1, step_seq=1, desc='Mark')

                else:
                    car.move(row=car.max_row_on_this_road, step_seq=1, desc='Mark')

                block_car = car
        return HAS_WAITING_CAR

    def move_cars_on_this_lane_in_step_2(self, lane_seq):
        """
        所有的车此时最前面应该都是waiting的状态，所以要将最前面的车标记成stop或者wait状态，最后一辆车就是该车道是否发生改变的标记
        该函数要配合car.move_to_next_road使用
        """
        block_car = None
        for car in self.car_arr[lane_seq, ::-1]:
            if car:
                if not car.is_waiting:
                    block_car = car
                    continue

                elif block_car is None:
                    if car.will_exceed_this_road:
                        car.wait()
                        self.push_head_car(car)
                    else:
                        car.move(row=car.max_row_on_this_road, step_seq=2, desc='Follow')
                        Data.to_schedule -= 1

                elif car.row + car.max_v_on_this_road >= block_car.row:
                    if block_car.is_waiting:
                        car.wait(block_car)
                    else:
                        car.move(row=block_car.row-1, block_dis=car.max_row_on_this_road-block_car.row+1, step_seq=2, desc='Follow')
                        Data.to_schedule -= 1
                else:
                    car.move(row=car.max_row_on_this_road, step_seq=2, desc='Follow')
                    Data.to_schedule -= 1

                block_car = car

