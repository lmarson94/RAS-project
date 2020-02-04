#!/usr/bin/env python
import rospy

import matplotlib.pyplot as plt
from ransac.msg import ransac_data, ransac_array
import numpy as np

np.random.seed(10)

def main():
    data = ransac_data()
    msg = ransac_array()
    data.x = 1.0
    data.y = 2.0
    msg.push_back(data)


    min_range = 0.0
    max_range = 5.0
    space = 0.1
    x = np.arange(min_range, max_range, space)
    y = np.ndarray(np.shape(x))
    xplot = np.arange(min_range, max_range, space)
    x[int(len(x)/2):] = -x[int(len(x)/2):] + 2 * min(x[int(len(x)/2):])
    y = x + np.random.normal(0, 0.05, len(x))

    xplot = np.array([-0.028021,
    1.316728,
    1.317806,
    0.907456,
    0.895806,
    0.886920,
    0.872616,
    0.860647,
    0.852838,
    0.836681,
    0.828621,
    0.808707,
    0.791283])
    y = np.array([1.850079,
    1.989360,
    1.983987,
    1.844780,
    1.827650,
    1.796519,
    1.776885,
    1.757389,
    1.723123,
    1.699993,
    1.662664,
    1.635276,
    1.608014])


    threshold_iter = len(xplot)
    threshold_dist = 0.02

    #plt.plot(xplot, x, label="true line")
    plt.scatter(xplot, y, label="noisy data", s=1)

    max_votes = 0
    iter = 0
    last_update = iter
    m_best, q_best = (0, 0)
    count=0
    while True:
        r1 = np.random.randint(0, len(xplot))
        r2 = np.random.randint(0, len(xplot))
        while r1 == r2:
            r2 = np.random.randint(0, len(xplot))
        count+=1
        (x1, y1) = (xplot[r1], y[r1])
        (x2, y2) = (xplot[r2], y[r2])

        m = (y1 - y2) / (x1 - x2)
        q = y1 - m * x1

        votes = 0

        for x_q, y_q in zip(xplot, y):
            if (x_q, y_q) == (x1, y1) or (x_q, y_q) == (x2, y2):
                continue


            x_p = (y_q + m * x_q - q) / (2 * m)
            y_p = m * x_p + q

            dist = np.sqrt((x_p - x_q)**2 + (y_p - y_q)**2)


            #dist = np.abs((y2 - y1) * x_q - (x2 - x1) * y_q + x2 * y1 - y2 * x1) / np.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)
            if dist < threshold_dist:
                votes += 1
        print(m, q)
        print(votes)
        if votes > max_votes:
            max_votes = votes
            m_best = m
            q_best = q
            last_update = iter

        if iter - last_update > threshold_iter:
            break

        iter += 1

    print("----------")
    print(m_best,q_best)


    plt.plot(xplot, [m_best * x_i + q_best for x_i in xplot], 'r', label="ransac 1 dist = " + str(threshold_dist))

    i = 0
    idx = []
    for x_q, y_q in zip(xplot, y):
        x_p = (y_q + m_best * x_q - q_best) / (2 * m_best)
        y_p = m_best * x_p + q_best

        dist = np.sqrt((x_p - x_q) ** 2 + (y_p - y_q) ** 2)
        if dist < threshold_dist:
            idx.append(i)

        i += 1

    xplot = np.delete(xplot, idx)
    y = np.delete(y, idx)
    print(idx)

    max_votes = 0
    iter = 0
    last_update = iter
    m_best, q_best = (0, 0)
    while True:
        r1 = np.random.randint(0, len(xplot))
        r2 = np.random.randint(0, len(xplot))
        while r1 == r2:
            r2 = np.random.randint(0, len(xplot))

        (x1, y1) = (xplot[r1], y[r1])
        (x2, y2) = (xplot[r2], y[r2])

        m = (y1 - y2) / (x1 - x2)
        q = y1 - m * x1

        votes = 0

        for x_q, y_q in zip(xplot, y):
            if (x_q, y_q) == (x1, y1) or (x_q, y_q) == (x2, y2):
                continue

            x_p = (y_q + m * x_q - q) / (2 * m)
            y_p = m * x_p + q

            dist = np.sqrt((x_p - x_q)**2 + (y_p - y_q)**2)

            if dist < threshold_dist:
                votes += 1

        if votes > max_votes:
            max_votes = votes
            m_best = m
            q_best = q
            last_update = iter

        if iter - last_update > threshold_iter:
            break

        iter += 1
        print(last_update, iter)

    print(m_best, q_best)
    #plt.plot(np.arange(min_range, max_range, space), [m_best * x_i + q_best for x_i in np.arange(min_range, max_range, space)], label="ransac 2 dist = " + str(threshold_dist))

    plt.legend()
    plt.show()

if __name__ == '__main__':
    rospy.init_node('ransac')
    try:
        main()
        array_pub = rospy.Publisher("/ransac_data", ransac_array, queue_size=10)
        array_pub.publish(msg)
    except rospy.ROSInterruptException:
        pass

rospy.spin()
