#-*-coding:utf-8-*- 
__author__ = 'MarkShawn'


from base import *

logger = logging.getLogger('core_judge')
if ENABLE_CORE_LOG:
    logger.addHandler(logging.FileHandler(filename='log_core_judge.log', mode='w'))
    logger.setLevel(logging.DEBUG)
else:
    logger.setLevel(logging.WARNING)



class CoreJudge(Base):

    """
    ===========================
    一次性初始化程序
    ===========================
    """


    def init_data(self):
        clear_file(LOG_FILE_PATH)
        self.read_cross_dict()          # 已传输CROSSES_CNT信息给Data
        self.read_road_dict()           # 已传输MAX_CARS_ON_MAP等信息给Data，并且已生成第一条最短路径
        self.read_car_dict()            # 已传输CARS_CNT信息给Data

    def pre_process_judging_data(self):
        self.read_answer_file()
        self.read_preset_answer_file()  # 自动加载预置小车信息，并配备出发时间和路径
        for car in self.car_dict.values():
            car.next_road.add_car_to_garage(car)

    """
    =======================
    模拟模式只有发车
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
        def _start_cars_in_garage(_priority):
            garage = road.prior_garage if _priority else road.normal_garage
            visited_but_failed_cars = []

            while not garage.empty():
                car = garage.get()

                if int(car.real_time) > time_round:
                    visited_but_failed_cars.append(car)
                    break

                elif road.is_blocking:
                    visited_but_failed_cars.append(car)
                    # logger.debug(
                    #     {'TimeRound': Data.time_round,
                    #      'Id': car.id,
                    #      'RealTime': car.real_time,
                    #      'Priority': car.priority,
                    #      'Preset': car.preset,
                    #      'Action': 'DelayedByBlocking',
                    #      })

                else:
                    blocked_car, max_pos = road.get_block_car_at_the_entrance(car.max_v_on_next_road)
                    if not max_pos:
                        car.delay()
                        visited_but_failed_cars.append(car)
                        # logger.debug(
                        #     {'TimeRound': Data.time_round,
                        #      'Id': car.id,
                        #      'RealTime': car.real_time,
                        #      'Priority': car.priority,
                        #      'Preset': car.preset,
                        #      'Action': 'DelayedByNoPos',
                        #      })
                    else:
                        car.start(*max_pos)
                        # logger.debug(
                        #     {'TimeRound': Data.time_round,
                        #      'Id': car.id,
                        #      'RealTime': car.real_time,
                        #      'Priority': car.priority,
                        #      'Preset': car.preset,
                        #      'Action': 'START',
                        #      })

            for car in visited_but_failed_cars:
                garage.put(car)

        _start_cars_in_garage(1)
        if not priority:
            _start_cars_in_garage(0)


