#-*-coding:utf-8-*- 
__author__ = 'MarkShawn'


from core import *
from utils import *


"""
==================================================================
装饰器 log_each_step   用于打印某个调度过程中发生的一些数据改变量
装饰器 log_each_round  用于打印某个时间片结束时的一些数据标量
==================================================================
"""


@log_each_step
def schedule_step_1(time_round):
    """
    第一步调度，标定道路上能移动或需要等待的小车
    该步骤结束时，将全图发优先车
    """
    if np.mod(time_round, REFRESH_FREQUENCY) == 1 : # and Data.total_finished_ratio < 0.9
        c.update_roads()  # 更新路权

    for road in c.road_dict.values():
        for lane_seq in range(road.car_arr.shape[0]):
            res = road.mark_cars_on_this_lane_in_step_1(lane_seq)
            if res and road not in Data.roads_to_schedule_in_step_2:
                Data.roads_to_schedule_in_step_2.append(road)

    c.load_cars(time_round=time_round, priority=True)
    c.depart_cars(time_round=time_round, priority=True)


@dead_lock_check # 死锁检测装饰器，服务器运行时自动休眠
def period_schedule_in_step_2(time_round):
    for road in list(Data.roads_to_schedule_in_step_2):
        while True:
            first_car = road.get_first_waiting_car()

            if not first_car:
                Data.roads_to_schedule_in_step_2.remove(road)
                break

            if c.car_is_conflicted(first_car) or not first_car.can_move_to_next_road:
                break

            lane = first_car.lane
            first_car.move_to_next_road()
            Data.to_schedule -= 1
            road.move_cars_on_this_lane_in_step_2(lane)
            c.depart_cars(time_round=time_round, priority=True, road=road)


@log_each_step
def schedule_step_2(time_round):
    """
    第二步调度，循环调度以使道路上所有的车移动至结束状态
    该步骤中，只要有小车通过路口，将继续对该小车原所在路做一次优先车发车
    该步骤结束后，将依次先全图发优先车，再全图发非优先车
    """

    while Data.roads_to_schedule_in_step_2:
        try:
            period_schedule_in_step_2(time_round=time_round)
        except:
            raise evaluate_results(c, succeed=False)

    c.depart_cars(time_round=time_round, priority=True)
    c.load_cars(time_round=time_round, priority=False)
    c.depart_cars(time_round=time_round, priority=False)


@log_each_round
def each_round(time_round):
    schedule_step_1(time_round)
    schedule_step_2(time_round)

    scan_and_log_cars_each_time_round(c, logger)


def main():
    while Data.total_finished != Data.CARS_CNT:
        Data.time_round += 1
        each_round(Data.time_round)

    if ENABLE_CAR_ARRIVAL_TIME_FILE_WRITE:
        write_arrival_times(c, 'arrival_core.txt')
        
    if ENABLE_EVALUATING_RESULTS_LOG:
        evaluate_results(c)
        

if __name__ == '__main__':
    c = Core()
    c.init_data()
    c.pre_process_data()
    main()
    write_answer(c, ANSWER_PATH, including_preset=False)



