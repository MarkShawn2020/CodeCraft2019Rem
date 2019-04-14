#-*-coding:utf-8-*- 
__author__ = 'MarkShawn'

import numpy as np


class Cross(object):

    def __init__(self, fake_id, real_id, *road_id_list):
        self.id = fake_id
        self.real_id = real_id
        self.road_id_list = road_id_list

    """
    ==================================
    以下函数用于道路ID和次序之间的转换与偏转
    ---基于所有的道路输入是按顺时针旋转的---
    ==================================
    """

    @property
    def road_id_to_seq_dict(self):
        return dict((j, i) for i, j in enumerate(self.road_id_list) if j != -1)

    @property
    def road_seq_to_id_dict(self):
        return dict((i, j) for i, j in enumerate(self.road_id_list) if j != -1)

    def get_offset_road_id(self, road_id, offset):
        offset_seq = np.mod(self.road_id_to_seq_dict[road_id] + offset, 4)
        return self.road_seq_to_id_dict.get(offset_seq)

    def get_offset_value_between_two_roads(self, this_road_id, next_road_id):
        return np.mod(self.road_id_to_seq_dict[next_road_id] - self.road_id_to_seq_dict[this_road_id], 4)

    """
    ===================================
    以下该函数用于绘图
    ===================================
    """

    def get_rotation_matrix(self, road_m, road_n):
        rot =  np.mod(self.road_id_to_seq_dict[road_n] - self.road_id_to_seq_dict[road_m], 4) * (- np.pi / 2)
        return np.matrix([[np.cos(rot), -np.sin(rot)], [np.sin(rot), np.cos(rot)]])




