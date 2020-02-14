# -*- coding: utf-8 -*-
import sys
import numpy as np
import matplotlib.pyplot as plt

def noise(average, deviation):
    return np.random.normal(average, deviation,(1,2))

def kalman_filter(w,r,t):
    #経過時間
    time = 0
    #更新単位時間
    dt = 1
    #初期速度
    u = 1.
    v = 3.
    #時刻end_timeまで予測する
    end_time = t
    #_trueは真の状態値,_estは推定値,_odoは予想値
    x_true = np.array([0.,0.])
    x_est = x_true
    x_odo = x_true
    v_base = np.array([u,v])
    v_est = v_base
    y_true = np.array([0.,0.])
    y_est = y_true
    variance = w**2
    #print(y_true)

    while time < end_time:
        #print(time)
        time += 1

        #予測値(odometry)の計算
        x_odo = x_est + v_est*dt

        #真値の計算
        v_noise = noise(0., w)
        v_true = v_base + v_noise
        x_true = x_true + v_true*dt
        y_true = np.vstack((y_true,x_true))
        # print(y_true[:,time])
        #観測値の計算
        error = noise(0.,r)
        x_obs = x_true + error


        #推定値(estimate)の計算

        #現在の状態の予測誤差の分散の計算
        variance = variance + w**2
        #カルマンゲインの計算
        k_gain = variance / (variance + r**2)
        #print(k_gain)
        #カルマンゲインで予測誤差の修正
        variance = (1 - k_gain)*variance
        #ケルマンゲインで予測値から推定値に修正
        x_est = x_odo + k_gain*(x_obs - x_odo)
        # print(x_est)
        y_est = np.vstack((y_est,x_est))
    #y_est_array = np.array(y_est).T
    # #y_true_array = np.array(y_true).T
    # y_true_nd = np.array(y_true).T
    # # y_est_nd = np.array(y_est).T
    # print(y_true_nd)
    #print(y_est)
    plt.plot(y_est[:,0],y_est[:,1],label = "acual move")
    plt.plot(y_true[:,0],y_true[:,1], linestyle = "--", label = "estimated move")
    plt.xlabel("x-coordinate")
    plt.ylabel("y-coordinate")

    plt.legend()
    plt.show()
    #print(y_est-y_true)

if __name__ == '__main__':
    args = sys.argv
    if 4 == len(args):
        kalman_filter(float(args[1]),float(args[2]),int(args[3]))
    else:
        print('Input Error')
