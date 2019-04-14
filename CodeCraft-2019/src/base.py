#-*-coding:utf-8-*- 
__author__ = 'MarkShawn'

from settings import *
from utils import *
from data import Data
from car import Car
from road import Road
from cross import Cross

from collections import OrderedDict, defaultdict, Counter
from queue import PriorityQueue
import numpy as np
import time


class Base():

    def __init__(self):

        self.start_time = time.time()

        self.cross_dict = None
        self.road_dict = None
        self.car_dict = None

        self.roads_to_schedule_in_step_2 = []           # 第一阶段调度后仍有等待车辆的道路列表，可提高程序性能
        self.car_queues = defaultdict(PriorityQueue)    # 四种车库，优先预置，非优先预置，非优先预置，非优先非预置

        self.MAX_PRESET_REAL_TIME = None

    @property
    def all_car_id_set(self):
        return set(self.car_dict)

    @property
    def prior_car_id_set(self):
        return set(i.id for i in self.car_dict.values() if i.priority == 1)

    @property
    def preset_car_id_set(self):
        return set(i.id for i in self.car_dict.values() if i.preset == 1)

    @property
    def CARS_CNT(self):
        return len(self.all_car_id_set)

    @property
    def PRIOR_CARS_CNT(self):
        return len(self.prior_car_id_set)

    @property
    def PRESET_CARS_CNT(self):
        return len(self.preset_car_id_set)

    @property
    def CROSSES_CNT(self):
        return len(self.cross_dict)

    @property
    def ROADS_CNT(self):
        return len(self.road_dict)

    @property
    def PAYLOAD_OF_MAP(self):
        return sum(i.payload for i in self.road_dict.values())

    @property
    def MAX_CARS_ON_MAP(self):
        return int(MAX_COVERAGE * self.PAYLOAD_OF_MAP)

    """
    =======================
    读取并初始化路口、道路、小车的字典信息，生成数据结构
    =======================
    """

    def read_cross_dict(self):
        cross_data = read_items_from_file(CROSS_PATH)
        sorted_cross_data = sorted(cross_data, key=lambda x: x[0])
        self.cross_dict = OrderedDict((i, Cross(i, *j)) for i, j in enumerate(sorted_cross_data))
        self.cross_real_id_to_id_dict = dict((j.real_id, j.id) for j in self.cross_dict.values())
        self.cross_id_to_real_id_dict = dict((j.id, j.real_id) for j in self.cross_dict.values())

        Data.CROSSES_CNT = self.CROSSES_CNT
        Data.init_arr()

        logging.info('MAP: {}'.format(MAP_NAME))
        logging.info('TotalCrosses: {}'.format(self.CROSSES_CNT))

    def read_road_dict(self):
        """
        并初始化最短路径
        :return:
        """
        self.road_dict = dict()
        self.road_id_to_vid_dict = dict()
        self.road_id_set = set()
        road_data = read_items_from_file(ROAD_PATH)
        for road_item in road_data:
            road_item[4] = self.cross_real_id_to_id_dict[road_item[4]]
            road_item[5] = self.cross_real_id_to_id_dict[road_item[5]]
            road = Road(*road_item)

            self.road_dict[road.vid] = road
            self.road_id_to_vid_dict[road_item[0]] = (road_item[4], road_item[5])
            self.road_id_set.add(road_item[0])

            Data.first_dis_arr[road.vid] = road.length / road.speed_limited

            if road.is_duplex:  # 因为会有双向路，而roads_container是按照道路的向量表示的，因此对于双向路要重建一个road类
                road_item[4], road_item[5] = road_item[5], road_item[4]
                road = Road(*road_item)

                self.road_dict[road.vid] = road

                Data.first_dis_arr[road.vid] = road.length / road.speed_limited

        Data.PAYLOAD_OF_MAP = self.PAYLOAD_OF_MAP
        Data.MAX_CARS_ON_MAP = self.MAX_CARS_ON_MAP
        Data.first_dis_arr /= Data.first_dis_arr[np.isfinite(Data.first_dis_arr)].max()
        Data.first_path_arr = get_path_arr(Data.first_dis_arr)  # 需要上一步生成的距离矩阵，并且生成路径矩阵
        Data.path_time_arr = self.get_path_time_arr(Data.first_path_arr)

        self.road_dict = OrderedDict(sorted(self.road_dict.items(), key=lambda x: (x[1].to_cross_id, x[1].id)))
        logging.info({'TotalRoads': self.ROADS_CNT,
                      'ByRoadID': len(road_data),
                      'PayRoadOfMap': self.PAYLOAD_OF_MAP,
                      'MAX_COVERAGE': MAX_COVERAGE,
                      'MAX_CARS_ON_MAP': self.MAX_CARS_ON_MAP
                      })

    def read_car_dict(self):
        car_data = read_items_from_file(CAR_PATH)
        car_dict = dict()
        for car_item in car_data:
            car_item[1] = self.cross_real_id_to_id_dict[car_item[1]]
            car_item[2] = self.cross_real_id_to_id_dict[car_item[2]]
            car_dict[car_item[0]] = car = Car(*car_item)

            if car.priority:
                Data.MAX_HIGH_TIME = max(Data.MAX_HIGH_TIME, car.real_time)

            car.time_need = Data.path_time_arr[car.vid]

        self.car_dict = car_dict


    def read_preset_answer_file(self):
        preset_data = read_items_from_file(PRESET_ANSWER_PATH)
        max_preset_real_time = 0
        for preset_item in preset_data:
            car = self.car_dict[preset_item[0]]
            car.real_time = preset_item[1]
            self.add_road_id_path(car, preset_item[2:])

            if car.priority:
                Data.MAX_HIGH_TIME = max(Data.MAX_HIGH_TIME, car.real_time)
        logging.info('\n\n')


    def read_answer_file(self):
        answer_data = read_items_from_file(ANSWER_PATH)
        for answer_item in answer_data:
            car = self.car_dict[answer_item[0]]
            car.real_time = answer_item[1]
            self.add_road_id_path(car, answer_item[2:])


    """
    ======================
    小车的方向与冲突检测
    ======================
    """

    def _get_car_dir(self, car):
        if not car.this_road or not car.next_road:
            return 2
        else:
            this_cross = self.cross_dict[car.this_road.to_cross_id]
            return this_cross.get_offset_value_between_two_roads(car.this_road.id, car.next_road.id)

    def _is_conflicted(self, car, data_list):
        cross_id = car.this_road.to_cross_id
        for data in data_list:
            offset, min_priority, dir = data
            road_id = self.cross_dict[cross_id].get_offset_road_id(car.this_road.id, offset)
            if road_id:
                road_vid = self.get_road_vid_from_cross_and_road_id(cross_id, road_id)
                if road_vid:
                    first_car = self.road_dict[road_vid].get_first_waiting_car()
                    if first_car and first_car.priority >= min_priority and self._get_car_dir(first_car) == dir:
                        # 这里还是加一个waiting_car 比较好
                        car.waiting_car = first_car
                        return True
        return False

    def car_is_conflicted(self, car):
        dir = self._get_car_dir(car)
        if car.priority == 1:
            if dir == 2:
                return False
            elif dir == 1:
                return self._is_conflicted(car, [(3, 1, 2)])                # 左转优先车等右手位直行优先车
            else:
                return self._is_conflicted(car, [(1, 1, 2), (2, 1, 1)])     # 右转优先车等左手位直行优先车或对手位左转优先车

        else:
            if dir == 2:
                return self._is_conflicted(car, [(1, 1, 1), (3, 1, 3)])     # 直行劣等车等左手位左转优先车或右手位右转优先车
            elif dir == 1:
                return self._is_conflicted(car, [(3, 0, 2), (2, 1, 3)])     # 左转劣等车等右手位直行任意车或对手位右转直行车
            else:
                return self._is_conflicted(car, [(1, 0, 2), (2, 0, 1)])     # 右转劣等车等左手位任意直行车或对手位任意左转车

    """
    ==============================
    以下是关于小车路径的各种转换与读写，支持三种加载小车路径的方法

    法一：在程序中生成小车的vid路径，vid是指道路的向量表示形式，API为self.add_road_vid_path(road_vid_path)
        调用这个API后会先生成road_vid_path，然后基于此通过道路字典生成road_cls_path与road_id_path

    法二：直接从答案文件加载，比如加载预置小车路径或者加载测试答案路径时，API为self.add_road_id_path(road_id_path)
        调用这个API后会内部通过self.convert_road_id_path_to_road_vid_path()转化成road_vid_path
        然后生成法一中的road_cls_path和road_id_path 
        
    法三：【更新于4月13号】动态路径直接更改 car.roads_to_go即可，已经实现，API在 core.update_roads()
    ==============================
    """


    def get_road_cls_path_from_vid_path(self, road_vid_path):
        return list(self.road_dict[i] for i in road_vid_path)


    def convert_road_id_path_to_road_vid_path(self, car, road_id_path):
        cross_id_path = [car.from_cross_id]
        for road_id in road_id_path:
            for next_cross_id in self.road_id_to_vid_dict[road_id]:
                if next_cross_id != cross_id_path[-1]:
                    cross_id_path.append(next_cross_id)
                    break

        road_vid_path = list((i, j) for i, j in zip(cross_id_path[:-1], cross_id_path[1:]))
        self.add_road_vid_path(car, road_vid_path)

    def add_road_id_path(self, car, input_road_id_path):
        car.road_id_path = list(input_road_id_path)
        self.convert_road_id_path_to_road_vid_path(car, car.road_id_path)

    def add_road_vid_path(self, car, input_road_vid_path):
        car.road_vid_path = input_road_vid_path
        car.road_cls_path = self.get_road_cls_path_from_vid_path(car.road_vid_path)
        car.roads_to_go = car.road_cls_path.copy()

        if not car.road_id_path:
            car.road_id_path = list(i.id for i in car.road_cls_path)


    """
    ===================================
    一些其他功能
    ===================================
    """

    def get_time_used(self, decimal=3, is_seconds=True):
        seconds = time.time() - self.start_time
        return round(seconds if is_seconds else seconds / 60, decimal)

    def get_road_real_vid(self, road):
        """
        该函数用于获得道路的真实向量
        """
        return (self.cross_id_to_real_id_dict[road.from_cross_id],
                self.cross_id_to_real_id_dict[road.to_cross_id],)

    def get_road_vid_from_cross_and_road_id(self, to_cross_id, road_id):
        """
        该函数用于路口判断，从路口ID和道路ID得到另一个路口的ID，并返回道路的向量。
        如果是单向路，则返回空
        """

        def swap_tuple(input_tuple):
            converted_list = list(input_tuple)
            converted_list[0], converted_list[1] = converted_list[1], converted_list[0]
            return tuple(converted_list)

        road_vid_tuple = self.road_id_to_vid_dict[road_id]
        road_vid_tuple = road_vid_tuple if road_vid_tuple[1] == to_cross_id else swap_tuple(road_vid_tuple)
        if road_vid_tuple in self.road_dict:
            return road_vid_tuple


    def get_path_time_arr(self, input_path_arr):
        """
        该函数用于将路径矩阵转换成时间矩阵，用于初始化小车形成的预估时间
        :param input_path_arr:
        :return:
        """
        def get_path_time(input_path):
            total_time = 0
            if input_path:
                for road_vid in input_path:
                    road = self.road_dict[road_vid]
                    total_time += int(road.length / road.speed_limited) + 1
            return total_time

        vfunc = np.vectorize(get_path_time)
        return vfunc(input_path_arr)


    def get_second_dis_path_arr(self):
        """
        该函数用于调整最频繁道路的路权，从而生成新的路径矩阵
        主要用于区分优先小车和普通小车的路径
        """
        road_list = [road_vid for ele in Data.first_path_arr.flatten() for road_vid in ele if ele]
        road_counter = Counter(road_list)
        max_freq = road_counter.most_common(1)[0][1]

        Data.second_dis_arr = Data.first_dis_arr.copy()
        for road_vid, road_freq in road_counter.items():
            road_freq_rate = road_freq / max_freq

            Data.second_dis_arr[road_vid] += road_freq_rate  * MULTIPLIER_TO_FIRST_ROAD


    """
    ==================================
    以下是评分函数
    ==================================
    """

    def get_some_property(self, is_prior, property, func):
        a = self.prior_car_id_set if is_prior else self.all_car_id_set
        b = [getattr(self.car_dict[i], property) for i in a]
        return func(b)

    def get_all_time_period(self):
        return Data.time_round

    def get_prior_time_period(self):
        return self.get_some_property(is_prior=True, property='finished_time', func=max) - \
               self.get_some_property(is_prior=True, property='plan_time', func=min)

    def get_all_scheduled_cnt(self):
        return sum(car.get_life_span() for car in self.car_dict.values())

    def get_prior_scheduled_cnt(self):
        return sum(car.get_life_span() for car in self.car_dict.values() if car.priority)

    def get_cars_cnt_ratio(self):
        return round(self.CARS_CNT / self.PRIOR_CARS_CNT, 5)

    def get_speed_ratio(self):
        numerator = round(self.get_some_property(False, 'speed', max) / self.get_some_property(False, 'speed', min), 5)
        denominator = round(self.get_some_property(True, 'speed', max) / self.get_some_property(True, 'speed', min), 5)
        return round(numerator / denominator, 5)

    def get_plan_time_ratio(self):
        numerator = round(
            self.get_some_property(False, 'plan_time', max) / self.get_some_property(False, 'plan_time', min), 5)
        denominator = round(
            self.get_some_property(True, 'plan_time', max) / self.get_some_property(True, 'plan_time', min), 5)
        return round(numerator / denominator, 5)

    def get_from_distributed_ratio(self):
        numerator = self.get_some_property(False, 'from_cross_id', lambda x: len(set(x)))
        denominator = self.get_some_property(True, 'from_cross_id', lambda x: len(set(x)))
        return round(numerator / denominator, 5)

    def get_to_distributed_ratio(self):
        numerator = self.get_some_property(False, 'to_cross_id', lambda x: len(set(x)))
        denominator = self.get_some_property(True, 'to_cross_id', lambda x: len(set(x)))
        return round(numerator / denominator, 5)

    def get_factor_from_weights_list(self, weights_list):
        return sum(i * j for i, j in zip(
            [self.get_cars_cnt_ratio(), self.get_speed_ratio(), self.get_plan_time_ratio(),
             self.get_from_distributed_ratio(), self.get_to_distributed_ratio()],
            weights_list
        ))

    def get_factor_a(self):
        return self.get_factor_from_weights_list([.05, .2375, .2375, .2375, .2375])

    def get_factor_b(self):
        return self.get_factor_from_weights_list([.8, .05, .05, .05, .05])

    def get_final_time_round(self):
        return int(round(self.get_factor_a() * self.get_prior_time_period() + self.get_all_time_period(), 0))

    def get_final_schedule_time(self):
        return int(round(self.get_factor_b() * self.get_prior_scheduled_cnt() + self.get_all_scheduled_cnt(), 0))


