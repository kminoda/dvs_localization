#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

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