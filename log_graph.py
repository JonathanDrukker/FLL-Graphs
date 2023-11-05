import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from math import degrees, pi
from csv import DictReader
import os


def log_graph(log, wheelRadius, DBM, waypoints=None):

    halfDBM = DBM / 2
    wheelCircumference = 2 * pi * wheelRadius

    # Reformatting log
    _log = {}
    for key, value in log[0].items():
        if isinstance(value, list) or isinstance(value, tuple):
            _log[key] = [[] for _ in range(len(value))]

    for timestamp in log:
        for key, value in timestamp.items():
            if isinstance(value, list) or isinstance(value, tuple):
                for i, subvalue in enumerate(value):
                    _log[key][i].append(subvalue)
            else:
                _log[key].append(value)
    _log = log

    # Creating plots
    pose_plot = plt.subplot2grid((3, 3), (0, 0), 2, 2)
    pose_plot.set_title("Pose")
    velocity_plot = plt.subplot2grid((3, 3), (2, 0), 2, 1)
    velocity_plot.set_title("Velocity")
    omega_plot = plt.subplot2grid((3, 3), (2, 1), 2, 1)
    omega_plot.set_title("Omega")
    theata_plot = plt.subplot2grid((4, 3), (0, 2), 1, 1)
    theata_plot.set_title("Theata")
    leftAcc_plot = plt.subplot2grid((8, 3), (2, 2), 1, 1)
    leftAcc_plot.set_title("Acc left", fontsize=8, y=0.9)
    leftAcc_plot.tick_params(bottom=False, labelbottom=False)
    rightAcc_plot = plt.subplot2grid((8, 3), (3, 2), 1, 1)
    rightAcc_plot.set_title("Acc right", fontsize=8, y=0.9)
    Vl_plot = plt.subplot2grid((4, 3), (2, 2), 1, 1)
    Vl_plot.set_title("Vl")
    Vr_plot = plt.subplot2grid((4, 3), (3, 2), 1, 1)
    Vr_plot.set_title("Vr")

    deltaTime = [t - log['time'][i-1] for i, t in enumerate(log['time'][1:])]

    if 'Pose' in log:
        points = np.array([log['x'], log['y']]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        lc = LineCollection(segments, cmap=plt.get_cmap('hsv'))
        lc.set_array(np.linspace(0, len(log['x']), len('x')))
        pose_plot.add_collection(lc)

        theata_plot.plot(log['time'], [degrees(i) for i in log['theata']])

        omega = [0]
        for i, theata in enumerate(log['theata'][1:]):
            if deltaTime[i] == 0:
                omega.append(omega[-1])
            else:
                if theata - log['theata'][i] > pi:
                    theata -= 2*pi
                elif theata - log['theata'][i] < -pi:
                    theata += 2*pi
                omega.append((theata - log['theata'][i]) / deltaTime[i])
        omega_plot.plot(log['time'], [degrees(i) for i in omega])

    if 'Phi' in log:
        Vl, Vr = [0], [0]
        for i, lPhi in enumerate(log['Phi'][0][1:]):
            Vl.append(lPhi/360 * wheelCircumference / deltaTime[i])
        for i, rPhi in enumerate(log['Phi'][1][1:]):
            Vr.append(rPhi/360 * wheelCircumference / deltaTime[i])

        Vl_plot.plot(log['time'], Vl)
        Vr_plot.plot(log['time'], Vr)

        velocity_plot.plot(log['time'], [sum(V)/2 for V in zip(Vl, Vr)])

        AccL, AccR = [0], [0]
        for i, V in enumerate(Vl[1:]):
            AccL.append((V - Vl[i]) / deltaTime[i])
        for i, V in enumerate(Vr[1:]):
            AccR.append((V - Vr[i]) / deltaTime[i])

        leftAcc_plot.plot(log['time'], AccL)
        rightAcc_plot.plot(log['time'], AccR)

    if waypoints:

        if 'x' in waypoints:
            points = np.array([log['waypoint']['x'], log['waypoint']['y']]).T.reshape(-1, 1, 2)
            segments = np.concatenate([points[:-1], points[1:]], axis=1)
            lc = LineCollection(segments, cmap=plt.get_cmap('hsv'))
            lc.set_array(np.linspace(0, len(log['waypoint']['x']), len(log['waypoint']['x'])))
            pose_plot.add_collection(lc)

        if 'V' in waypoints:
            velocity_plot.plot(log['time'], log['waypoint']['V'])

        if 'omega' in waypoints:
            omega_plot.plot(log['time'], [degrees(i) for i in log['waypoint']['omega']])

        if 'theata' in waypoints:
            theata_plot.plot(log['time'], [degrees(i) for i in log['waypoint']['theata']])

        if 'leftAcc' in waypoints:
            leftAcc_plot.plot(log['time'], log['waypoint']['leftAcc'])

        if 'rightAcc' in waypoints:
            rightAcc_plot.plot(log['time'], log['waypoint']['rightAcc'])

        if 'V' in waypoints and 'omega' in waypoints:
            Vl_plot.plot(log['time'], [V - omega*halfDBM for V,
                                       omega in zip(log['waypoint']['V'],
                                                    log['waypoint']['omega'])])

            Vr_plot.plot(log['time'], [V + omega*halfDBM for V,
                                       omega in zip(log['waypoint']['V'],
                                                    log['waypoint']['omega'])])

    pose_plot.autoscale_view()
    plt.subplots_adjust(left=0.03, bottom=0.04, right=0.985, top=0.96, wspace=0.125, hspace=0.5)
    plt.show()


def main():

    print("Starting...")

    wheelRadius = 4.075
    DBM = 9.7
    pathname = '1'

    path = os.path.dirname(os.getcwd()) + "FLL/"

    with open(f'{path}/Robot/Paths/{pathname}.csv', 'r') as f:
        waypoints = [i for i in DictReader(f)]

    with open(path+'Data/Logs/runtime.log', 'r') as f:
        log_graph([i for i in DictReader(f)], wheelRadius, DBM, waypoints)

    print("Done!")


if __name__ == '__main__':
    main()
