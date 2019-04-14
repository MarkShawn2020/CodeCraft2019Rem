#-*-coding:utf-8-*- 
__author__ = 'MarkShawn'


from base import *
import heapq
from collections import Counter

logger = logging.getLogger('core')
if ENABLE_CORE_LOG:
    logger.addHandler(logging.FileHandler(filename='log_core.log', mode='w'))
    logger.setLevel(logging.DEBUG)
else:
    logger.setLevel(logging.WARNING)


class Core(Base):

    """
    ===========================
    一次性初始化程序
    ===========================
    """

    def transfer_Data(self):
        logging.info({'TotalCars': self.CARS_CNT,
                      'PriorCars': self.PRIOR_CARS_CNT,
                      'PresetCars': self.PRESET_CARS_CNT,
                      'IntersectedCars': len(self.prior_car_id_set.intersection(self.preset_car_id_set))
                      })

        Data.CARS_CNT = self.CARS_CNT
        Data.PRIOR_CARS_CNT = self.PRIOR_CARS_CNT
        Data.PRESET_CARS_CNT = self.PRESET_CARS_CNT
        Data.MAX_CARS_ON_MAP = self.MAX_CARS_ON_MAP

    def init_data(self):
        clear_file(LOG_FILE_PATH)
        self.read_cross_dict()          # 已传输CROSSES_CNT信息给Data
        self.read_road_dict()           # 已传输MAX_CARS_ON_MAP等信息给Data，并且已生成第一条最短路径
        self.read_car_dict()            # 已传输CARS_CNT信息给Data

    def pre_process_data(self):
        self.read_preset_answer_file()  # 自动加载预置小车信息，并配备出发时间和路径

        self.get_second_dis_path_arr()

        self.change_preset_property()

        self.transfer_Data()

        self.pure_sort_cars(self.preset_car_id_set)
        self.sort_prior_cars(self.prior_car_id_set-self.preset_car_id_set, log_k=ARG_K_PRIOR)
        self.sort_cars(self.all_car_id_set - self.preset_car_id_set - self.prior_car_id_set, log_k=ARG_K_NON_PRIOR, update_time=True)



    """
    =======================
    三种排车方案
    =======================
    """
    def touch_top(self):
        cars = [self.car_dict[i] for i in self.preset_car_id_set]
        car_info_list = [i.real_time for i in cars]
        counter = Counter(car_info_list)
        return

    def change_preset_property(self):
        cars = [self.car_dict[i] for i in self.preset_car_id_set]
        sorted_cars = sorted(cars, key=lambda x: (- x.time_need, -x.priority, -x.speed))
        car_cnt = len(cars)
        max_cars_to_change = int(car_cnt/10)

        for seq, car in enumerate(sorted_cars):
            if seq < max_cars_to_change:
                car.preset = 0
            else:
                break

    def pure_sort_cars(self, car_id_set):
        """
        静态装载小车，预置小车专用，因为它已经有出发时间和路径信息
        """
        for car_id in car_id_set:
            car = self.car_dict[car_id]
            self.car_queues[car.priority, car.preset].put(car)

        return

    def sort_cars(self, car_id_set, log_k=2, update_time=True):
        """
        log_k: 排车的log对数底，数值越大，说明排车越靠前，系数设置为2时接近下降阶梯分布
        Prior:  1, x
        Preset: x, 1
        """
        car_list = sorted([self.car_dict[i] for i in car_id_set],
                          key=lambda x: (x.real_time, -x.speed, -x.real_time - x.time_need, x.id))
        car_cnt = len(car_list)

        ALL_MAX_TIME = self.get_some_property(is_prior=False, property='real_time', func=max)

        cur_sum_cnt = 0
        for time_round in range(1, ALL_MAX_TIME + 1):
            max_sum_cnt = np.log(time_round / ALL_MAX_TIME * (log_k-1)+1) / np.log(log_k) * car_cnt
            while car_list and car_list[0].real_time <= time_round and cur_sum_cnt<max_sum_cnt:
                car = car_list.pop(0)
                cur_sum_cnt += 1
                if update_time:
                    car.real_time = time_round
                self.car_queues[car.priority, car.preset].put(car)

        return

    def sort_prior_cars(self, car_id_set, log_k=3):
        """
        log_k: 排车的log对数底，数值越大，说明排车越靠前，系数设置为2时接近下降阶梯分布
        """

        car_list = []
        for car_id in car_id_set:
            car = self.car_dict[car_id]
            heapq.heappush(car_list, (car.real_time, -car.real_time-car.time_need, -car.speed, car.id, car.time_need))

        car_cnt = len(car_list)

        sorted_car_tuple_list = []
        cur_sum_cnt = 0
        for time_round in range(1, Data.MAX_HIGH_TIME + 1):
            max_sum_cnt = np.log(time_round / Data.MAX_HIGH_TIME * (log_k-1)+1) / np.log(log_k) * car_cnt
            while car_list:
                real_time, arrival_time, speed, car_id, time_need = car_info = heapq.heappop(car_list)
                if car_info[0] <= time_round and cur_sum_cnt<= max_sum_cnt:
                    cur_sum_cnt += 1
                    sorted_car_tuple_list.append(car_info)
                elif car_info[0] <= time_round:
                    heapq.heappush(car_list, (time_round+1, -real_time-time_need, speed, car_id, time_need))
                else:
                    heapq.heappush(car_list, car_info)
                    break
            else:
                break

        for real_time, arrival_time, speed, car_id, time_need in sorted_car_tuple_list:
            car = self.car_dict[car_id]
            car.real_time = real_time
            self.car_queues[car.priority, car.preset].put(car)

        return

    """
    =======================
    更新路权
    =======================
    """
    def update_roads(self):
        Data.path_arr_for_prior = get_path_arr(Data.first_dis_arr + Data.road_use_arr * MULTIPLIER_TO_TRAFFIC)
        Data.path_arr_for_non_prior = get_path_arr(Data.second_dis_arr + Data.road_use_arr * MULTIPLIER_TO_TRAFFIC)

        Data.update_path_arr_cnt += 1
        logging.debug('---------- Updated Shortest Path Matrix {} Times -------------'.format(Data.update_path_arr_cnt))
        #
        for car in Data.car_set_on_map:
            if car.preset == 0 and car.next_road and not car.will_exceed_this_road:
                new_roads_vid_to_go = Data.path_arr_for_prior[(car.this_road.to_cross_id, car.to_cross_id)]
                if new_roads_vid_to_go[0] != tuple(reversed(car.this_road.vid)) and new_roads_vid_to_go:
                    car.roads_to_go = [self.road_dict[road_vid] for road_vid in new_roads_vid_to_go]

    """
    =======================
    装车
    =======================
    """

    def load_cars(self, time_round, priority=True):
        """
        动态加载小车，并更新其出发时间、路径等信息
        """

        while not self.car_queues[priority, 1].empty():     # 延迟预置车的唯一条件，就是预置车车库中的车，实际出发时间要大于当前时间

            car = self.car_queues[priority, 1].get_nowait()
            if car.real_time > time_round:
                self.car_queues[priority, 1].put(car)
                break
            else:
                self.load_car_into_garage(car)

        while not self.car_queues[priority, 0].empty():
            car = self.car_queues[priority, 0].get_nowait()

            if car.real_time > time_round or Data.total_to_start >= MAX_TO_START:
                """
                对于非预置车的装车，不但有发车时间限制，还加入了车库数量的限制，这样能保证发车阶段遍历车库时有较高的性能
                """
                self.car_queues[priority, 0].put(car)
                break

            else:
                """
                对优先车和非优先车分别使用两种道路权值，尽量使他们的路径避开
                """
                path_arr = None
                if car.priority:
                    path_arr = Data.path_arr_for_prior

                elif Data.prior_finished_ratio >= 0.99 and car.speed > SPEED_GROUP_CUT:
                    path_arr = Data.path_arr_for_prior

                else:
                    path_arr = Data.path_arr_for_non_prior

                self.load_car_into_garage(car, sync_time=True, new_path_arr=path_arr)

    def load_car_into_garage(self, car, new_path_arr=None, sync_time=False):
        if sync_time is True:
            car.real_time = Data.time_round
        if new_path_arr is not None:
            self.add_road_vid_path(car, new_path_arr[car.vid])
        car.next_road.add_car_to_garage(car)


    """
    =======================
    发车
    =======================
    """

    def depart_cars(self, time_round, priority=True, road=None):
        """
        先上优先车，再上劣等车
        其中，有预置车就必上，否则可以进行一些判断，以自定义调节一些发车
        """
        if road:
            self.start_cars_in_garage(road=road, time_round=time_round, priority=priority)

        else:
            for road in self.road_dict.values():
                self.start_cars_in_garage(road=road, time_round=time_round, priority=priority)

    def start_cars_in_garage(self, road, time_round, priority):

        def start():
            car.start(*max_pos, block_dis=car.max_v_on_next_road - max_pos[1])

        def wait():
            car.delay(non_preset_sync_time=True)
            cars_failed_to_start.append(car)

        garage = road.prior_garage if priority else road.normal_garage
        cars_failed_to_start = []

        while not garage.empty():
            car = garage.get()

            if int(car.real_time) > time_round: # 如果从车库内pop出来的车，出发时间大于当前时间，迭代结束
                cars_failed_to_start.append(car)
                break

            elif road.is_blocking:  # 当前道路已经完全被堵
                wait()

            else:
                blocked_car, max_pos = road.get_block_car_at_the_entrance(car.max_v_on_next_road)
                if not max_pos:         # 若有正在等待的车阻挡（可能自己速度较快），则pop下一辆可以上路的车（可能速度较慢点就可以上路了）
                    wait()

                elif car.preset:        # 预置车上路不受任何限制，只要有路就上
                    start()

                elif max_pos[0] == road.lanes_cnt-1 and max_pos[1] < 1:
                    wait()

                elif car.priority:
                    start()

                elif Data.cars_on_map < Data.MAX_CARS_ON_MAP:
                    start()

                else:
                    wait()

        for car in cars_failed_to_start:
            garage.put(car)

