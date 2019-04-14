#-*-coding:utf-8-*- 
__author__ = 'MarkShawn'

import logging
import sys


MAP_PATH = '..'

"""
复赛测试地图
2-map-training-1
2-map-training-2

小数据测试地图
2-training-training-1-answer
2-training-training-2-answer
"""

# MAP_NAME = '2-training-training-1-answer'
# MAP_NAME = '2-map-training-1'
MAP_NAME = '2-map-exam-2'
#

ENABLE_EVALUATING_RESULTS_LOG = True
ENABLE_EACH_ROUND_LOG = True if len(sys.argv) < 2 else False
ENABLE_DEAD_LOCK_CHECK = True

ENABLE_CORE_LOG = False                 # core[_judge]
ENABLE_CAR_LOG = False                  # car
ENABLE_EACH_CAR_POS_LOG = False         # utils.scan_and_log_cars_each_time_round()
ENABLE_CAR_ARRIVAL_TIME_FILE_WRITE = False

# MAX_STD_LOAD = 0.15
# MAX_PRIOR_TO_START = 300

MAX_COVERAGE = 0.14                 # 地图上最大车辆覆盖率，如果使用纯动态规划可达0.20，保险0.15，半动态能到0.10，纯静态0.05
REFRESH_FREQUENCY = 3               # 必须大于1，每N时间片更新一次道路权重（包含优先路径和非优先路径），越短越耗性能

MULTIPLIER_TO_FIRST_ROAD = 10       # 相对于最常使用道路的罚权倍数，场景：非优先车对于优先车的躲避
MULTIPLIER_TO_TRAFFIC = 10          # 相对于当前交通流量的罚权倍数，场景：每N时间片

PRIOR_FINISH_CUT = 0.99             # 优先小车完成多少比例时，给速度快的小车赋更短的路程
SPEED_GROUP_CUT = 8                 # 定义车速的分界点

MAX_TO_START = 200                  # 车库中最大的车辆保有数目，每次发车都要遍历车库内所有的车，因此这个数字越小性能越高，但也意味着最优车辆越稀缺，进而导致全地图车辆数目较少

ARG_K_PRIOR = 1.5                     # 优先车排车的对数参数，2为平缓下降解读，数值越大，前段时间排的车越多
ARG_K_NON_PRIOR = 1.5   # ???                 # 非优先车排车的对数参数，由于非优先车很多，一般取2及以下，避免早期大量拥挤





















if len(sys.argv) == 6:
    MAP_PATH = '.'
    LOG_FILE_PATH = '../logs/CodeCraft-2019.log'
    CAR_PATH, ROAD_PATH, CROSS_PATH, PRESET_ANSWER_PATH, ANSWER_PATH = sys.argv[1:]
    MAP_NAME = CAR_PATH.split('/')[0].split('\\')[0]

else:
    LOG_FILE_PATH = '../../logs/CodeCraft-2019.log'
    CAR_PATH, ROAD_PATH, CROSS_PATH, PRESET_ANSWER_PATH, ANSWER_PATH = \
        map(lambda x: '{}/{}/{}.txt'.format(MAP_PATH, MAP_NAME, x), ('car', 'road', 'cross', 'presetAnswer', 'answer'))

logging.basicConfig(level=logging.DEBUG,
                    filename=LOG_FILE_PATH,
                    format='[%(asctime)s] %(levelname)s [%(funcName)s: %(filename)s, %(lineno)d] %(message)s',
                    datefmt='%Y-%m-%d %H:%M:%S',
                    filemode='a')

logging.info('MAP: {}'.format(MAP_NAME))