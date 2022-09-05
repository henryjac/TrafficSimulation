import math
import random
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
import matplotlib.pyplot as plt
from matplotlib import animation, cm
import numpy as np
import matplotlib

def python():
    fig = plt.figure()
    ax = fig.gca()
    t = np.linspace(-0.3,1,1000)
    x = np.sqrt(t**3-t**2+3)
    y = t

    ax.plot(x,y)
    plt.legend(("First","Second"))

    plt.show()

def find_car():
    i = 0
    j = 0
    while True:
        try:
            if traffic_obj.cars[car_nr+i].lane == car.lane:
                car_ahead = traffic_obj.cars[car_nr+i]
                break
            i += 1
        except IndexError: # Reached the end of the car array, start from beginning again
            if traffic_obj.cars[j].lane == car.lane:
                car_ahead = traffic_obj.cars[j]
                break
            j += 1
        if j > 100000:
            print("Something terrible happened when trying to integrate car {}. Traffic:".format(car_nr))
            print(traffic_obj)
            exit()
    return car_ahead