import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from math import pi, degrees
from json import loads
import os


def log_graph(log, wheelRadius, DBM, waypoints=None):

    halfDBM = DBM / 2
    wheelCircumference = 2 * pi * wheelRadius

    # Reformatting log
    tmp = log
    log = {"time": [], "x": [], "y": [], "theata": [], "V": [[], []], "VlT": [], "VrT": []}
    for spline in tmp:
        for timestamp in spline:
            log["time"].append(timestamp[0])
            log["x"].append(timestamp[1])
            log["y"].append(timestamp[2])
            log["theata"].append(timestamp[3])
            log["V"][0].append(timestamp[4][0])
            log["V"][1].append(timestamp[4][1])
            log["VlT"].append(timestamp[5])
            log["VrT"].append(timestamp[6])

    # Reformatting waypoints
    if waypoints:
        tmp = waypoints
        waypoints = {"time": [], "x": [], "y": [], "theata": [], "V": [], "omega": [], "accL": [], "accR": []}
        for waypoint in tmp:
            waypoints["time"].append(float(waypoint[0]))
            waypoints["x"].append(float(waypoint[1]))
            waypoints["y"].append(float(waypoint[2]))
            waypoints["theata"].append(float(waypoint[3]))
            waypoints["V"].append(float(waypoint[4]))
            waypoints["omega"].append(float(waypoint[5]))
            waypoints["accL"].append(float(waypoint[6]))
            waypoints["accR"].append(float(waypoint[7]))

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

    if 'x' in log:
        points = np.array([log['x'], log['y']]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        lc = LineCollection(segments, cmap=plt.get_cmap('hsv'))
        lc.set_array(np.linspace(0, len(log['x']), len(log['x'])))

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

    if 'V' in log:

        Vl = [i/360 * wheelCircumference for i in log['V'][0]]
        Vr = [i/360 * wheelCircumference for i in log['V'][1]]

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

    if 'VlT' in log:
        Vl_plot.plot(log['time'], log['VlT'])
        Vr_plot.plot(log['time'], log['VrT'])

    if waypoints:

        if 'x' in waypoints:
            points = np.array([waypoints['x'], waypoints['y']]).T.reshape(-1, 1, 2)
            segments = np.concatenate([points[:-1], points[1:]], axis=1)
            lc = LineCollection(segments, cmap=plt.get_cmap('hsv'))
            lc.set_array(np.linspace(0, len(waypoints['x']), len(waypoints['x'])))
            pose_plot.add_collection(lc)

        if 'V' in waypoints:
            velocity_plot.plot(waypoints['time'], waypoints['V'])

        if 'omega' in waypoints:
            omega_plot.plot(waypoints['time'], [degrees(i) for i in waypoints['omega']])

        if 'theata' in waypoints:
            theata_plot.plot(waypoints['time'], [degrees(i) for i in waypoints['theata']])

        if 'accL' in waypoints:
            leftAcc_plot.plot(waypoints['time'], waypoints['accL'])

        if 'accR' in waypoints:
            rightAcc_plot.plot(waypoints['time'], waypoints['accR'])

        if 'V' in waypoints and 'omega' in waypoints:
            Vl_plot.plot(waypoints['time'], [V - omega*halfDBM for V,
                                             omega in zip(waypoints['V'],
                                                          waypoints['omega'])])

            Vr_plot.plot(waypoints['time'], [V + omega*halfDBM for V,
                                             omega in zip(waypoints['V'],
                                                          waypoints['omega'])])

    pose_plot.autoscale_view()
    plt.subplots_adjust(left=0.03, bottom=0.04, right=0.985, top=0.96, wspace=0.125, hspace=0.5)
    plt.show()


def main():

    print("Starting...")

    pathname = '1'

    path = os.path.dirname(os.getcwd()) + "\\FLL"

    with open(f'{path}\\Robot\\config.json', 'r') as f:
        config = loads(f.read())

    wheelRadius = config["wheel"]["radius"]
    DBM = config["drivebase"]["DBM"]

    with open(f'{path}\\Robot\\Paths\\{pathname}.path', 'r') as f:
        waypoints = []
        for spline in eval(f.read()):
            for waypoint in spline:
                waypoints.append(waypoint)

    with open(path+'\\Data\\Logs\\runtime.log', 'r') as f:
        log_graph(eval(f.read()), wheelRadius, DBM, waypoints)

    print("Done!")


if __name__ == '__main__':
    main()
