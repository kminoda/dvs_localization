#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# fig = plt.figure()

# ims = []

# for i in range(10):
#         rand = np.random.randn(100)     # 100個の乱数を生成
#         im = plt.plot(rand)             # 乱数をグラフにする
#         ims.append(im)                  # グラフを配列 ims に追加

# # 10枚のプロットを 100ms ごとに表示
# ani = animation.ArtistAnimation(fig, ims, interval=100)
# plt.show()

ims = []

xs = np.arange(0, 5, 1)
ys = np.arange(0, 5, 1)

X = []
Y = []
for x in xs:
    for y in ys:
        X.append(x)
        Y.append(y)
fig = plt.figure(figsize=(15, 15))

im1 = plt.scatter(X, Y, s=3000, marker='o', color='white')
ims.append([im1])

im2 = plt.scatter(X, Y, s=3000, marker='o', color='black')
ims.append([im2])

ani = animation.ArtistAnimation(fig, ims, interval=2)
plt.show()