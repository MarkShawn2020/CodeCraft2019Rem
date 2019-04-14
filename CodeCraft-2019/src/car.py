#-*-coding:utf-8-*- 
__author__ = 'MarkShawn'

from data import *
from settings import *

import logging
logger = logging.getLogger('car')
if ENABLE_CAR_LOG:
    logger.addHandler(logging.FileHandler(filename='log_car_judge.log', mode='w'))
    logger.setLevel(logging.DEBUG)
else:
    logger.setLevel(logging.WARNING)



class Car(object):
    def __init__(self, car_id, car_from, car_to, car_v, plan_time, priority, preset):
        self.id = car_id
        self.from_cross_id = car_from
        self.to_cross_id = car_to
        self.speed = car_v
        self.plan_time = plan_time
        self.priority = priority
        self.preset = preset

        self.vid = (self.from_cross_id, self.to_cross_id)

        self._real_time = self.plan_time
        self._start_time = self.plan_time
        self.finished_time = self.plan_time

        self.road_id_path = None
        self.road_vid_path = None
        self.roads_passed = []
        self.roads_to_go = []
        # self.visited_cross_id_set = {self.from_cross_id}

        self.pos_track = [self.id]

        self.row = None
        self.lane = None

        self.is_waiting = False
        self.has_finished = False

        self.waiting_car = None
        self.follow_car = None

        self.time_need = None


    """
    ====================================
    小车的优先队列判定函数
    ====================================
    """

    def __lt__(self, other):
        """
        由于在程序中使用了优先对列PriorityQueue，且直接对小车的类进行排序
        因此定义该函数，在内部实现基于优先级、发车时间、ID的排序
        这样能够保证在神奇车库内发车时，总是吻合官方方案的（前提是判别器的路面判断没有太大误差）
        :param other: 与该辆小车比较顺序的另外一辆小车
        :return: 返回True即该小车的优先级较高，否则反之
        """

        if self.priority > other.priority:
            return True
        elif self.priority < other.priority:
            return False

        elif int(self.real_time) < int(other.real_time):
            return True
        elif int(self.real_time) > int(other.real_time):
            return False

        elif self.id < other.id:
            return True
        elif self.id > other.id:
            return False

        else:
            logging.error('Cant compare between these two cars: {}, {}'.format(self.id, other.id))


    """
    ====================================
    小车的一些时间属性
    ====================================
    """
    @property
    def real_time(self):
        return self._real_time

    @real_time.setter
    def real_time(self, value):
        """
        由于官方会有对发车时间与计划时间的检测
        因此我们定义了Setter的方法，检测发车时间的有效性
        """
        if value < self.plan_time:
            logging.error({'car_id': self.id, 'preset': self.preset, 'plan_time': self.plan_time, 'rel_time': value, })
        else:
            self._real_time = value
            self._start_time = self._real_time

    @property
    def start_time(self):
        return self._start_time

    @start_time.setter
    def start_time(self, value):
        self._start_time = value

    @property
    def due_finished_time(self):
        return self.start_time + self.time_need

    def get_life_span(self):
        return self.finished_time - self.plan_time


    """
    ====================================
    小车的一些位置属性
    ====================================
    """

    @property
    def current_pos(self):
        return (self.lane, self.row, self.this_road.id)

    @property
    def this_road(self):
        if self.roads_passed:
            return self.roads_passed[-1]

    @property
    def next_road(self):
        if self.roads_to_go:
            return self.roads_to_go[0]

    @property
    def max_v_on_this_road(self):
        return min(self.speed, self.this_road.speed_limited)

    @property
    def max_v_on_next_road(self):
        return min(self.speed, self.next_road.speed_limited)

    @property
    def remaining_dis_on_this_road(self):
        return self.this_road.length - self.row - 1

    @property
    def max_dis_on_next_road(self):
        if not self.this_road:
            return self.max_v_on_next_road
        elif self.next_road:
            return max(self.max_v_on_next_road - self.remaining_dis_on_this_road, 0)
        else:
            return None

    @property
    def max_row_on_this_road(self):
        return min(self.row + self.max_v_on_this_road, self.this_road.length - 1)

    @property
    def will_exceed_this_road(self):
        return self.row + self.max_v_on_this_road >= self.this_road.length


    """
    ==========================================
    以下是小车的两种位置标记函数
    ==========================================
    """

    def _clear_current_position(self):
        self.is_waiting = False

        self.this_road.del_car(self)

    def _update_new_position(self, lane, row):

        self.lane = lane
        self.row = row
        self.pos_track.append(self.current_pos)
        self.this_road.add_car(self)


    """
    ==========================================
    以下判断头部小车能否过路口，以及过路口的函数
    注意！这里没有考虑小车冲突的问题，冲突会在主调度中检测！
    ==========================================
    """
    @property
    def can_move_to_next_road(self):
        if not self.next_road:
            return True
        elif self.next_road.is_blocking:
            return True
        else:
            block_car_on_next_road, max_pos = self.next_road.get_block_car_at_the_entrance(self.max_dis_on_next_road)
            if block_car_on_next_road and block_car_on_next_road.is_waiting:
                # 这里要补一句这个waiting_car，以生成waiting chain
                self.waiting_car = block_car_on_next_road
                return False
            else:
                return True

    def move_to_next_road(self):
        """
        专门给要通过路口的小车使用的，建立在self.can_move_to_next_road==True的基础上
        该函数要配合road.move_cars_on_this_lane_in_step_2使用
        """
        self.this_road.pop_head_car(self)

        if not self.next_road:
            return self.finish()

        elif self.next_road.is_blocking or self.max_dis_on_next_road == 0:
            return self.move(row=self.max_row_on_this_road, step_seq=2, desc='Cross')

        else:
            block_car_on_next_road, max_pos = self.next_road.get_block_car_at_the_entrance(self.max_dis_on_next_road)
            return self.move(*max_pos, next_road=True, block_dis=self.max_row_on_this_road-max_pos[1]+1, step_seq=2, desc='Cross')



    """
    ==========================================
    以下是小车的五种运动函数
    ==========================================
    """

    @stat_data
    def start(self, lane, row, block_dis=0):

        self.is_waiting = False
        self.start_time = Data.time_round
        self.roads_passed.append(self.roads_to_go.pop(0))
        self._update_new_position(lane, row)
        self.block_dis = block_dis

        return 1

    @stat_data
    def finish(self):
        self.finished_time = Data.time_round
        self.has_finished = True
        self._clear_current_position()
        return 2

    @stat_data
    def move(self, lane=None, row=None, next_road=False, block_dis=0, follow_car=None, step_seq=None, desc=None):
        self._clear_current_position()

        if lane is None:
            lane=self.lane
        if row is None:
            logging.error('Row is necessary!')
        if next_road:
            self.roads_passed.append(self.roads_to_go.pop(0))
            # self.visited_cross_id_set.add(self.this_road.to_cross_id)

        self._update_new_position(lane, row)
        self.is_waiting = False
        self.follow_car = follow_car
        self.block_dis = block_dis

        return 3

    @stat_data
    def wait(self, waiting_car=None):
        self.waiting_car = waiting_car
        self.is_waiting = True
        return 4

    @stat_data
    def delay(self, non_preset_inc_time=False, non_preset_sync_time=False):
        """
        Delay的时候非预置车可以考虑将实际出发时间+1，但往往不需要，尤其是模拟模式下根本不要
        """
        if non_preset_inc_time and self.preset==0:
            self.real_time += 1
        if non_preset_sync_time and self.preset == 0:
            self.real_time = Data.time_round + 1

        return 5