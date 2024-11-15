from math import radians, degrees, tan, atan2, pi, fabs

def cot(a):
    return 1. / tan(a)

def calculate_angle(x, y):
    angle_rad = atan2(y, x)
    angle_deg = angle_rad * (180/pi)
    angle_deg = (angle_deg + 360) % 360
    return angle_deg

def specialCaseAlgorithm(point_i, point_j, point_k, Tij):
    xi_ = point_i[0] - point_j[0]
    yi_ = point_i[1] - point_j[1]

    xk_ = point_k[0] - point_j[0]
    yk_ = point_k[1] - point_j[1]

    xij_ = xi_ + Tij*yi_
    yij_ = yi_ - Tij*xi_

    xjk_ = xk_ + Tij*yk_
    yjk_ = yk_ - Tij*xk_

    xki_ = yk_ - yi_
    yki_ = xi_ - xk_

    Kki_ = xi_*yk_ - xk_*yi_

    D = (xjk_ - xij_)*(yki_) + (yij_ - yjk_)*(xki_)

    xr = point_j[0] + (Kki_*(yij_ - yjk_)/D)
    yr = point_j[1] + (Kki_*(xjk_ - yij_)/D)

    return xr, yr

def triangleAlgorithm(ls_3_pointMap, ls_3_pointRef):
    angle_1 = radians(calculate_angle(ls_3_pointRef[0][0], ls_3_pointRef[0][1]))
    angle_2 = radians(calculate_angle(ls_3_pointRef[1][0], ls_3_pointRef[1][1]))
    angle_3 = radians(calculate_angle(ls_3_pointRef[2][0], ls_3_pointRef[2][1]))

    # print(angle_1, angle_2, angle_3)

    # -- Xác định hệ tọa độ mới của gƣơng
    x1_ = ls_3_pointMap[0][0] - ls_3_pointMap[1][0]
    y1_ = ls_3_pointMap[0][1] - ls_3_pointMap[1][1]

    x3_ = ls_3_pointMap[2][0] - ls_3_pointMap[1][0]
    y3_ = ls_3_pointMap[2][1] - ls_3_pointMap[1][1]

    Phi12 = angle_2 - angle_1
    Phi23 = angle_3 - angle_2

    # -- Tính giá trị các cot(.)
    T12 = cot(Phi12)
    T23 = cot(Phi23)

    T31 = (1 - T12*T23)/(T12+T23)

    # -- Xác định tọa độ tâm gƣơng thay thế
    x12_ = x1_ + T12*y1_
    y12_ = y1_ - T12*x1_

    x23_ = x3_ - T23*y3_
    y23_ = y3_ + T23*x3_

    x31_ = (x3_ + x1_) + T31*(y3_ - y1_)
    y31_ = (y3_ + y1_) - T31*(x3_ - x1_)

    # -- Xác định hệ số đặc trƣng của đƣờng tròn
    k31_ = x1_*x3_ + y1_*y3_ + T31*(x1_*y3_ - x3_*y1_)

    # -- Tính định thức
    D = (x12_ - x23_)*(y23_ - y31_) - (y12_ - y23_)*(x23_ - x31_)

    # -- Xác định vị trí và hƣớng của robot
    xr = ls_3_pointMap[1][0] + (k31_*(y12_ - y23_)/D)
    yr = ls_3_pointMap[1][1] + (k31_*(x23_ - x12_)/D)

    phiR = atan2(ls_3_pointMap[1][1] - yr, ls_3_pointMap[1][0] - xr) - angle_2

    return xr, yr, phiR
