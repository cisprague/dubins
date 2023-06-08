# Christopher Iliffe Sprague
# sprague@kth.se

import numpy as np

# check if three points are on same line
def onseg(p, q, r):

    # extract points
    px, py = p
    qx, qy = q
    rx, ry = r

    # conditions
    c0 = qx <= max(px, rx)
    c1 = qx >= min(px, rx)
    c2 = qy <= max(py, ry)
    c3 = qy >= min(py, ry)

    # return
    if c0 and c1 and c2 and c3:
        return True
    else:
        return False

# orientation of triplet
def orientation(p, q, r):

    # extract points
    px, py = p
    qx, qy = q
    rx, ry = r

    # orientation
    val = (qy - py)*(rx -qx) - (qx - px)*(ry - qy)

    # if colinear
    if val == 0:
        return 0
    # if clockwise
    elif val > 0:
        return 1
    # if counterclockwise
    else:
        return 2

# test intersection
def intersection(seg0, seg1):

    # extract points
    p0 = seg0[0, :]
    q0 = seg0[1, :]
    p1 = seg1[0, :]
    q1 = seg1[1, :]

    # orientations
    o0 = orientation(p0, q0, p1)
    o1 = orientation(p0, q0, q1)
    o2 = orientation(p1, q1, p0)
    o3 = orientation(p1, q1, q0)

    # general intersection
    if o0 != o1 and o2 != o3:
        return True
    # sepcial colinear cases
    elif o0 == 0 and onseg(p0, p1, q0):
        return True
    elif o1 == 0 and onseg(p0, q1, q0):
        return True
    elif o2 == 0 and onseg(p1, p0, q1):
        return True
    elif o3 == 0 and onseg(p1, q0, q1):
        return True
    else:
        return False

# intersection points
def intersection_point(seg1, seg2):

    # extract points
    x1, y1 = seg1[0, :]
    x2, y2 = seg1[1, :]
    x3, y3 = seg2[0, :]
    x4, y4 = seg2[1, :]

    # x
    d1  = np.linalg.det([[x1, y1], [x2, y2]])
    d2  = np.linalg.det([[x1, 1], [x2, 1]])
    d3  = np.linalg.det([[x3, y3], [x4, y4]])
    d4  = np.linalg.det([[x3, 1], [x4, 1]])
    d5  = np.linalg.det([[x1, 1], [x2, 1]])
    d6  = np.linalg.det([[y1, 1], [y2, 1]])
    d7  = np.linalg.det([[x3, 1], [x4, 1]])
    d8  = np.linalg.det([[y3, 1], [y4, 1]])
    num = np.linalg.det([[d1, d2], [d3, d4]])
    den = np.linalg.det([[d5, d6], [d7, d8]])
    x   = num/den

    # y - shares some determinants
    d2  = np.linalg.det([[y1, 1], [y2, 1]])
    d4  = np.linalg.det([[y3, 1], [y4, 1]])
    num = np.linalg.det([[d1, d2], [d3, d4]])
    y = num/den

    # return intersection point
    return np.array([x, y], float)




if __name__ == '__main__':

    # example lines
    line0 = np.array([[0, 10], [10, 10]])
    line1 = np.array([[5, 10.1], [8, 15]])

    # test interection
    print(intersection(line0, line1))

    # visualise
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1)
    ax.plot(line0[:,0], line0[:,1], 'r.-')
    ax.plot(line1[:,0], line1[:,1], 'g.-')
    ax.plot(*intersection_point(line0, line1), 'kx')
    plt.show()
