#!/usr/bin/env python3

import numpy as np
from PIL import Image

# 创建400x400的地图
width = 400
height = 400
map_data = np.full((height, width), 255, dtype=np.uint8)

# 添加外墙
wall_thickness = 10
map_data[0:wall_thickness, :] = 0  # 上墙
map_data[-wall_thickness:, :] = 0  # 下墙
map_data[:, 0:wall_thickness] = 0  # 左墙
map_data[:, -wall_thickness:] = 0  # 右墙

# 添加一些内部结构
# 横向墙
map_data[150:160, 100:300] = 0
# 纵向墙
map_data[200:350, 250:260] = 0

# 添加一些随机障碍物
np.random.seed(42)
for _ in range(20):
    x = np.random.randint(50, width-50)
    y = np.random.randint(50, height-50)
    size = np.random.randint(10, 30)
    map_data[y-size//2:y+size//2, x-size//2:x+size//2] = 0

# 保存为PGM文件
image = Image.fromarray(map_data)
image.save('maps/test_map.pgm') 