#-*-coding:utf-8-*- 
__author__ = 'MarkShawn' 

import settings
from settings import *
from data import Data

from scipy.sparse.csgraph import shortest_path
import numpy as np
import logging
import time
import sys
import re


def clear_file(file_path):
    with open(file_path, 'w') as f:
        f.write('#\n')

def read_items_from_file(file):
    with open(file, 'r') as f:
        return [list(map(int, re.findall(r'-?\d+', i))) for i in f.readlines() if not i.startswith('#')]

def write_arrival_times(c, car_arrival_time_file):
    sorted_cars = sorted(c.car_dict.values(), key=lambda x: (x.finished_time, x.id))
    with open(car_arrival_time_file, 'w') as f:
        for car in sorted_cars:
            f.write('({}, {})\n'.format(car.id, car.finished_time))

def get_path_arr(dis_arr):
    """
    :param dis_arr: 输入距离矩阵
    :return: 返回路径列表（每条道路都时向量形式）
    """

    def convert_path_arr_to_road_vid_path_arr(path_arr):
        path_list_arr = np.empty(path_arr.shape, dtype=object)
        for row_seq, row in enumerate(path_arr):
            for col_seq, cell in enumerate(row):
                path_list = [col_seq]
                while True:
                    path_node = row[path_list[0]]
                    if path_node >= 0:
                        path_list.insert(0, path_node)
                    else:
                        break
                path_list_arr[row_seq, col_seq] = [(i, j) for i, j in zip(path_list[:-1], path_list[1:])]
        return path_list_arr

    dis_arr, path_node_arr = shortest_path(dis_arr, return_predecessors=True,)
    path_arr = convert_path_arr_to_road_vid_path_arr(path_node_arr)


    return path_arr


def dead_lock_check(func):
    def wrapper(*args, **kwargs):
        """
        输入调度步骤二每个周期开始时的累计小车移动数量，与结束时的累计小车移动数量作比较
        如果相同，且确实还有在等待的小车，则代表这个周期内没有移动小车，即发生了死锁
        """

        def yield_next_first_waiting_car(car):
            waiting_car = car.waiting_car
            waiting_road = waiting_car.this_road
            next_waiting_car = waiting_road.get_first_waiting_car()
            yield waiting_car
            yield next_waiting_car
            # waiting car == next_waiting_car 的情况是指冲突的时候
            if next_waiting_car == waiting_car or next_waiting_car not in visited_cars_set:
                visited_cars_set.add(next_waiting_car)
                yield from yield_next_first_waiting_car(next_waiting_car)
            else:
                return

        to_schedule = Data.to_schedule
        visited_cars_set = set()
        a = func(*args, **kwargs)

        if ENABLE_DEAD_LOCK_CHECK:
            if Data.to_schedule != 0 and Data.to_schedule==to_schedule:
                road = next(iter(Data.roads_to_schedule_in_step_2))
                start_car = road.get_first_waiting_car()

                waiting_chain = list(map(lambda x: (x.id, x.priority, x.current_pos, x.this_road.vid)
                ,[start_car] + [i for i in yield_next_first_waiting_car(start_car)]))

                while waiting_chain[0] != waiting_chain[-1]:
                    waiting_chain.pop(0)


                from pprint import pformat
                logging.error('\n{}'.format(pformat(waiting_chain, indent=4)))
                logging.error('Dead Lock!')

                raise Exception('Dead Lock!')
        return a
    return wrapper


def log_each_step(func):
    def wrapper(*args, **kwargs):
        start_time = time.time()
        data_old = Data.__dict__.copy()
        a = func(*args, **kwargs)
        if ENABLE_EACH_ROUND_LOG:
            data_new = Data.__dict__
            data_changed = {'TimeUsed': round(time.time() - start_time, 3)}
            data_changed.update(dict((i, data_new[i]-data_old[i]) for i, j in data_old.items() if isinstance(j, (int, float)) and data_new[i] != data_old[i]))
            logging.info(data_changed)
        return a
    return wrapper



def log_each_round(func):
    def wrapper(*args, **kwargs):
        a = func(*args, **kwargs)
        if ENABLE_EACH_ROUND_LOG:
            logging.info('\nTimeRound: {}, CarsOnMap: {}, PriorOnMap: {}, PresetOnMap: {}, NormalOnMap: {}\n'
                         'TotalStarted: {}, PriorToStart: {}, PresetToStart: {}, TotalToStart: {}, PriorStarted: {}, PresetStarted: {}\n'
                         'Coverage: {:.2f}%, PriorFinishedRatio: {:.2f}%, PresetFinishedRatio: {:.2f}%, TotalFinishedRatio: {:.2f}%\n'
                         'TimeRun: {:.3f}s, DueTime: {:.3f}s\n'.format(
                Data.time_round, Data.cars_on_map, Data.prior_cars_on_map, Data.preset_cars_on_map, Data.normal_cars_on_map,
                Data.total_started, Data.prior_to_start, Data.preset_to_start, Data.total_to_start, Data.prior_started, Data.preset_started,
                Data.cars_coverage * 100, Data.prior_finished_ratio*100, Data.preset_finished_ratio*100, Data.total_finished_ratio*100,
                Data.get_run_time(), Data.get_run_time()*Data.CARS_CNT / (Data.total_finished + 1) ))
            # logging.info({'StdEachRoad': np.std(Data.road_use_arr[np.nonzero(Data.road_use_arr)]).round(2)})
        return a
    return wrapper


def write_answer(c, answer_file, including_preset=False):
    with open(answer_file, 'w') as f:
        f.write('#\n')
        for car in c.car_dict.values():
            if (including_preset and car.preset) or not including_preset:
                f.write('({}, {}, {})\n'.format(car.id, int(car.real_time if car.preset else car.start_time), str([i.id for i in car.roads_passed])[1:-1]))



def scan_and_log_cars_each_time_round(c, logger):
    if ENABLE_EACH_CAR_POS_LOG:
        for road in c.road_dict.values():
            road_cars = [car.current_pos for car in road.car_arr.flatten() if car]
            if road_cars:
                logger.info({'TimeRound': Data.time_round, 'RoadId': road.id, 'RoadCars': road_cars})




"""
==================================
评分函数的打印
==================================
"""

def evaluate_results(c, succeed=True):
    """
    本函数用于生成最后的输出结果，因此程序是基于模拟的动态判别器，所以结果具有较高的准确度，但还没有完全准确（或者说其实还相差不少）
    :return:
    """
    logger = logging.getLogger('EVALUATE_RESULTS')
    fh = logging.FileHandler(filename='{}/{}/result.txt'.format(MAP_PATH, MAP_NAME), mode='a')
    logger.addHandler(fh)

    info_1 = {
        'MAX_COVERAGE': settings.MAX_COVERAGE,
        'MAX_TO_START': settings.MAX_TO_START,
        'K_PRIOR': settings.ARG_K_PRIOR,
        'K_NON_PRIOR': settings.ARG_K_NON_PRIOR,
        'REFRESH_FREQUENCY': settings.REFRESH_FREQUENCY,
        'M1': settings.MULTIPLIER_TO_FIRST_ROAD,
        'M2': settings.MULTIPLIER_TO_TRAFFIC,
        'MapName': MAP_NAME,
        'TimeUsed': time.time()-c.start_time,
    }

    info_2 = {
        'T_Prior': c.get_prior_time_period(),
        'T_Total': c.get_all_time_period(),
        'T_Final': c.get_final_time_round(),
        'S_Prior': c.get_prior_scheduled_cnt(),
        'S_Total': c.get_all_scheduled_cnt(),
        'S_Final': c.get_final_schedule_time(),
        'Factor_A': c.get_factor_a(),
        'Factor_B': c.get_factor_b(),
    }

    logger.info('\n{}'.format('='* 20))
    logger.info(info_1)
    logger.info(info_2)
    if not succeed:
        logger.info('=====  FAILED  !!! ======')
    else:
        logger.info('===== Succeeded!!! ======')

