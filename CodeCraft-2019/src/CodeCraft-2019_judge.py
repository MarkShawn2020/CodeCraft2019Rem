#-*-coding:utf-8-*- 
__author__ = 'MarkShawn'

from core_judge import *
from utils import *


@log_each_step
def schedule_step_1(time_round):

    for road in c.road_dict.values():
        for lane_seq in range(road.car_arr.shape[0]):
            res = road.mark_cars_on_this_lane_in_step_1(lane_seq)
            if res and road not in c.roads_to_schedule_in_step_2:
                Data.roads_to_schedule_in_step_2.append(road)

    c.depart_cars(time_round=time_round, priority=True)


@dead_lock_check
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

    while Data.roads_to_schedule_in_step_2:
        period_schedule_in_step_2(time_round=time_round)

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
        write_arrival_times(c, 'arrival_judge.txt')

    if ENABLE_EVALUATING_RESULTS_LOG:
        evaluate_results(c)


if __name__ == '__main__':
    c = CoreJudge()
    c.init_data()
    c.pre_process_judging_data()
    main()
    write_answer(c, '{}/{}/my_answer.txt'.format(MAP_PATH, MAP_NAME))

