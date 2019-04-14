# #-*-coding:utf-8-*-
# __author__ = 'MarkShawn'
#
#

"""
绘制发车分布情况
"""
import matplotlib.pyplot as plt
import seaborn as sns

queue = self.car_queues[0, 0]
qlist = []
while not queue.empty():
    qlist.append(queue.get())


tlist = [i.real_time for i in qlist]
sns.distplot(tlist)
plt.show()




"""
打印车库里队列信息
"""
while True:
    queue = c.car_queues[1, 0]
    if not queue.empty():
        car = queue.get_nowait()
        print(car.real_time, car.speed, car.id)
    else:
        break


"""
读取并排序答案
"""
import re
def read_items_from_file(file):
    with open(file, 'r') as f:
        return [list(map(int, re.findall(r'-?\d+', i))) for i in f.readlines() if not i.startswith('#')]
data_list = read_items_from_file('answer.txt')
sorted_list = sorted(data_list, key=lambda x: (x[1], x[0]))

data_list_2 = read_items_from_file('my_answer.txt')
sorted_list_2 = sorted(data_list_2, key=lambda x: (x[1], x[0]))

print(sorted_list == sorted_list_2)




"""
读取两个判别器的日志并对比
"""

with open('log_car_core.log', 'r') as f1:
     with open('log_car_judge.log', 'r') as f2:
         seq = 0
         while True:
             seq += 1
             s1 = f1.readline()
             s2 = f2.readline()
             if s1 != s2:
                 print(seq, s1, s2)
                 break


"""
分析答案中是否存在环
"""

from core_judge import *

def check_ring(car):
    visited_nodes = []
    nodes_list = [road.to_cross_id for road in car.roads_to_go]
    HAS_RING = False
    for i in nodes_list:
        if i not in visited_nodes:
            visited_nodes.append(i)
        else:
            HAS_RING = True
    return HAS_RING


if __name__ == '__main__':
    c = CoreJudge()
    c.init_data()
    c.read_answer_file()

    ring_cars = list(filter(check_ring, c.car_dict.values()))

    print('Finished')