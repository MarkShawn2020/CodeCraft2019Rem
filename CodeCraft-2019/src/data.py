#-*-coding:utf-8-*-
__author__ = 'MarkShawn'


import numpy as np
import time


class classproperty(object):
    """
    =======================
    该类只是为提供直接的类属性
    =======================
    """
    def __init__(cls, f):
        cls.f = f
    def __get__(cls, obj, owner):
        return cls.f(owner)


class Data():

    start_time = time.time()

    time_round = 0
    scheduled = 0
    prior_scheduled = 0

    to_schedule = 0
    update_path_arr_cnt = 0
    car_set_on_map = set()

    total_loaded = 0
    prior_loaded = 0
    preset_loaded = 0

    total_started = 0
    prior_started = 0
    preset_started = 0
    normal_started = 0

    total_finished = 0
    prior_finished = 0
    preset_finished = 0
    normal_finished = 0

    total_moved = 0
    prior_moved = 0
    preset_moved = 0

    total_waited = 0
    prior_waited = 0
    preset_waited = 0

    total_delayed = 0
    prior_delayed = 0
    preset_delayed = 0

    CARS_CNT = None
    CROSSES_CNT = None
    PRESET_CARS_CNT = None
    PRIOR_CARS_CNT = None
    MAX_CARS_ON_MAP = None
    PAYLOAD_OF_MAP = None

    MAX_HIGH_TIME = 0

    road_use_arr = None
    road_block_arr = None
    path_time_arr = None
    first_dis_arr = None
    first_path_arr = None
    second_dis_arr = None
    second_path_arr = None
    path_arr_for_prior = None
    path_arr_for_non_prior = None

    roads_to_schedule_in_step_2 = []

    @classmethod
    def init_arr(cls):
        cls.road_use_arr = np.zeros((cls.CROSSES_CNT, cls.CROSSES_CNT), dtype=float)
        cls.road_block_arr = np.zeros((cls.CROSSES_CNT, cls.CROSSES_CNT), dtype=float)

        cls.first_dis_arr = np.zeros((cls.CROSSES_CNT, cls.CROSSES_CNT), dtype=float)
        cls.first_dis_arr.fill(np.inf)
        cls.first_path_arr = np.empty((cls.CROSSES_CNT, cls.CROSSES_CNT), dtype=object)

        cls.second_dis_arr = np.zeros((cls.CROSSES_CNT, cls.CROSSES_CNT), dtype=float)
        cls.second_dis_arr.fill(np.inf)
        cls.second_path_arr = np.empty((cls.CROSSES_CNT, cls.CROSSES_CNT), dtype=object)

        cls.path_arr_for_prior = np.empty((cls.CROSSES_CNT, cls.CROSSES_CNT), dtype=object)
        cls.path_arr_for_non_prior = np.empty((cls.CROSSES_CNT, cls.CROSSES_CNT), dtype=object)

    @classmethod
    def get_run_time(self, decimal=3, is_seconds=True):
        seconds = time.time() - self.start_time
        return round(seconds if is_seconds else seconds / 60, decimal)

    @classproperty
    def cars_on_map(cls):
        return cls.total_started - cls.total_finished

    @classproperty
    def cars_coverage(cls):
        return round(cls.cars_on_map / cls.PAYLOAD_OF_MAP, 5)

    @classproperty
    def prior_cars_on_map(cls):
        return cls.prior_started - cls.prior_finished

    @classproperty
    def preset_cars_on_map(cls):
        return cls.preset_started - cls.preset_finished

    @classproperty
    def normal_cars_on_map(cls):
        return cls.normal_started - cls.normal_finished

    @classproperty
    def prior_finished_ratio(cls):
        return round(cls.prior_finished / cls.PRIOR_CARS_CNT, 5)

    @classproperty
    def preset_finished_ratio(cls):
        return round(cls.preset_finished / cls.PRESET_CARS_CNT, 5)

    @classproperty
    def total_finished_ratio(cls):
        return round(cls.total_finished / cls.CARS_CNT, 5)

    @classproperty
    def prior_to_start(cls):
        return cls.prior_loaded - cls.prior_started

    @classproperty
    def preset_to_start(cls):
        return cls.preset_loaded - cls.preset_started

    @classproperty
    def total_to_start(cls):
        return cls.total_loaded - cls.total_started



def stat_data(func):
    """
    ==================================================================
    装饰器 stat_Data 收集每辆小车的运动状态信息，用于跟踪、反馈与分析
    ==================================================================
    1: start                小车出发上路
    2: finish               小车抵达了目的地
    3: move                 移动，或去下一条路
    4: wait                 小车等待
    5: delay                小车延迟一秒出发，分车满被阻挡和主动延迟两种

    """
    def wrapper(self, *args, **kwargs):
        a = func(self, *args, **kwargs)
        if a == 1:
            Data.total_started += 1

            if self.priority:
                Data.prior_started +=1

            if self.preset:
                Data.preset_started += 1

            if not self.priority and not self.preset:
                Data.normal_started += 1

            Data.car_set_on_map.add(self)
            # Data.road_block_arr[self.this_road.vid] += self.block_dis / self.this_road.payload


        elif a == 2:
            Data.total_finished += 1

            if self.priority:
                Data.prior_finished += 1

            if self.preset:
                Data.preset_finished += 1

            if not self.priority and not self.preset:
                Data.normal_finished += 1

            Data.car_set_on_map.remove(self)    # 这里竟然填了add?

        elif a == 3:
            Data.total_moved += 1

            # Data.road_block_arr[self.this_road.vid] += self.block_dis / self.this_road.payload

        elif a == 4:
            Data.total_waited += 1

        elif a == 5:
            Data.total_delayed += 1

            if self.priority:
                Data.prior_delayed += 1

            if self.preset:
                Data.preset_delayed += 1


        return a
    return wrapper


