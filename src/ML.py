# Henry Jacobson, Simulation and Modeling, KTH 2020

#################### Imports ####################
import math
import time
import random
import matplotlib.pyplot as plt
from matplotlib import animation, cm
import numpy as np
import matplotlib
try:
    import xmltodict
except ImportError:
    print("Can't import xmltodict")
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)
#################################################

################### Set seed ####################
np.random.seed(666)
random.seed(666)

np.random.seed(667)
random.seed(667)
#################################################

################## Logging #####################
import logging
from logging.config import fileConfig
logging.config.fileConfig("../config/log.conf")
logger = logging.getLogger("pyLog")
#################################################

################## Traffic ######################
class Traffic():
    """ Defines a whole traffic model. """

    def __init__(self, gen_params, params, idm=True, max_start_vel=True, define_cars_per_lane=True):
        """ params needs to be an array with all the parameters defined
            in the IDM / CA"""

        if define_cars_per_lane:
            road_length = gen_params[0]; n_cars_per_lane = gen_params[1]
            max_speed   = gen_params[2]; n_lanes         = gen_params[3]
            lanes, start_positions, start_velocities, max_speed_car, counter_array =\
                get_pos_vel1(road_length, n_cars_per_lane, max_speed, n_lanes,\
                idm=idm, max_start_vel=max_start_vel)
        else:
            road_length = gen_params[0]; n_cars  = gen_params[1]
            max_speed   = gen_params[2]; n_lanes = gen_params[3]
            if idm:
                lanes, start_positions, start_velocities, max_speed_car, counter_array =\
                    get_pos_vel2(road_length, n_cars, max_speed, n_lanes,\
                    idm=idm, car_length=params[5])
            else:
                lanes, start_positions, start_velocities, max_speed_car, counter_array =\
                    get_pos_vel2(road_length, n_cars, max_speed, n_lanes,\
                    idm=idm)

        # Initialie all car objects
        vCar = np.vectorize(Car)
        self.cars = vCar(lanes, start_positions, start_velocities, max_speed_car, counter_array)

        # Initialize traffic parameters
        if idm==True:
            self.a  = params[0]; self.delta = params[1]; self.s0 = params[2]
            self.T  = params[3]; self.b     = params[4]; self.d  = params[5]
            self.gamma_max  = params[6]
            self.gamma_min  = params[7]
            self.gamma_safe = params[8]
            self.overtake_time = params[9]

            self.model = "IDM"
        else:
            self.p = params[0]
            self.model = "CA"

        self.road_length = road_length
        if define_cars_per_lane:
            self.n_cars      = n_cars_per_lane * n_lanes
        else:
            self.n_cars = n_cars
        self.n_lanes     = n_lanes

    def __str__(self):
        string  = "_____________________________________________________________________\n"
        string += "|   Car    |   Lane   |   Position   |   Velocity   |   Max Speed   |\n"
        string += "‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\n"
        string += "_____________________________________________________________________\n"
        for car in self.cars:
            string += str(car) + "\n"
        string += "‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\n"
        # string += "With model parameters:"
        # string += "\n a     = {}\n delta = {}\n s0    = {}\n".format(self.a, self.delta, self.s0)
        # string += " T     = {}\n b     = {}\n d     = {}".format(self.T, self.b, self.d)
        return string

    def find_next_car(self, lane, pos, ahead=True, same_lane=True):
        """ Find the next car ahead/behind in the same lane from pos """
        first = True
        for car_i in self.cars:
            if not car_i.lane != lane:
                if car_i.p == pos and car_i.lane == lane and same_lane:
                    index_same_car = car_i.cnt - 1
                    continue
                if ahead:
                    dist = (car_i.p - pos) % self.road_length
                else:
                    dist = (self.road_length - (car_i.p - pos) % self.road_length) % self.road_length
                if first:
                    index = car_i.cnt - 1
                    extreme_value = dist
                    first = False
                elif dist < extreme_value:
                    index = car_i.cnt - 1
                    extreme_value = dist

        try:
            car_ahead = self.cars[index]
        except UnboundLocalError:
            try:
                car_ahead = self.cars[index_same_car]
            except UnboundLocalError:
                car_ahead = None
        return car_ahead

    def drive_by1_idm(self, car):
        """ Check if car should begin overtaking next car"""
        # Get next car in same lane and check if time to change lane
        car_ahead = self.find_next_car(car.lane, car.p, ahead=True, same_lane=True)
        if  car.lane != self.n_lanes\
            and (car_ahead.p - car.p) % self.road_length < self.gamma_max * self.d\
            and (car_ahead.p - car.p) % self.road_length > self.gamma_min * self.d\
            and car.v > car_ahead.v:

            # Get cars in next lane to see if there is room for lane change
            car_ahead_next_lane = self.find_next_car(car.lane+1, car.p, ahead=True, same_lane=False)
            car_behind_next_lane = self.find_next_car(car.lane+1, car.p, ahead=False, same_lane=False)

            if (car_ahead_next_lane is None and car_behind_next_lane is None) or\
                ((car_ahead_next_lane.p - car.p) % self.road_length > self.gamma_safe * self.d\
                and (car.p - car_behind_next_lane.p) % self.road_length > self.gamma_safe * self.d):

                car.lane += 1
                lane_change = True
                car.start_driveby(self.overtake_time)
                return car_ahead_next_lane, lane_change

        lane_change = False
        return car_ahead, lane_change

    def drive_by2_idm(self, car):
        """ Check if car is finished overtaking. """
        if car.lane != 1 and car.until_driveby <= 0:
            car_ahead_previous_lane = self.find_next_car(car.lane-1, car.p, ahead=True)
            car_behind_previous_lane = self.find_next_car(car.lane-1, car.p, ahead=False)

            if (car_ahead_previous_lane is None and car_behind_previous_lane is None)\
                or ((car_ahead_previous_lane.p - car.p) % self.road_length > self.gamma_safe * self.d\
                and (car.p - car_behind_previous_lane.p) % self.road_length > self.gamma_safe * self.d):

                car.lane -= 1
                return car_ahead_previous_lane
        return None

    def i_fnc(self, v_n, x_n, v_n0, v_n1, x_n1):
        car_dist = (x_n1 - x_n) % self.road_length
        delta_vn = v_n - v_n1

        # Free traffic induced acceleration (fti)
        a_fti = 1 - (v_n / v_n0)**self.delta

        # If the car we are looking at is the only car in that lane
        if car_dist == 0:
            return self.a * a_fti

        # Car induced acceleration (ci)
        s_n      = car_dist - self.d
        s_n_star = self.s0 + v_n * self.T
        if delta_vn > 0:
            sqrt_ab_term = (v_n * delta_vn) / (2 * np.sqrt(self.a * self.b))
            s_n_star += sqrt_ab_term
        a_ci = (s_n_star / s_n)**2

        acc = self.a * (a_fti - a_ci)
        return acc

    def integrate_system(self, integrator):
        theta = []
        r = []
        for car_i in range(1,len(self.cars)+1):
            integrator.integrate(self, car_i)

            theta.append(self.cars[car_i-1].p * 2 * math.pi / self.road_length)
            r.append(-self.cars[car_i-1].lane * 0.1 + 1.1)
        return theta, r
#################################################

#################### Car ########################
class Car():
    """ A car in traffic with a specific lane, velocity, position and
        comfortable maximum speed."""

    def __init__(self, lane, position, velocity, max_speed, counter):
        self.t = 0
        self.lane = lane
        self.p = position
        self.v = velocity

        self.v_max = max_speed
        self.cnt = counter # Will also be used as color

        self.obs = Observables()
        self.obs.time.append(self.t)
        self.obs.lane.append(self.lane)
        self.obs.pos.append(self.p)
        self.obs.vel.append(self.v)

        self.until_driveby = 0

    def __str__(self):
        string = "|   {:3d}    |   {:4d}   |   {:.8s}   |   {:.8s}   |   {:.9s}   |"\
            .format(self.cnt, self.lane, "{:0.6f}".format(self.p), "{:0.6f}".format(self.v), "{:0.7f}".format(self.v_max))
        return string

    def start_driveby(self, overtake_time):
        self.until_driveby = overtake_time / Integrator.dt
#################################################

################ Observables ####################
class Observables():
    def __init__(self):
        self.time = []
        self.lane = []
        self.pos = []
        self.vel = []
#################################################

################## Integrators ##################
class Integrator():
    dt = 0.1

    def integrate(self, traffic_obj, car_nr):
        self.timestep(traffic_obj, car_nr)

        car = traffic_obj.cars[car_nr-1]
        car.obs.time.append(car.t)
        car.obs.lane.append(car.lane)
        car.obs.pos.append(car.p)
        car.obs.vel.append(car.v)

    def timestep(self, traffic_obj, car_nr):
        """ Implemented by child classes"""
        pass

class RK4Integrator(Integrator):
    def timestep(self, traffic_obj, car_nr):
        # Get the current car
        car = traffic_obj.cars[car_nr-1]

        car_ahead, lane_change = traffic_obj.drive_by1_idm(car)
        if not lane_change:
            pot_car_ahead = traffic_obj.drive_by2_idm(car)
            if pot_car_ahead is not None:
                car_ahead = pot_car_ahead

        # Define necessary velocities and positions
        v_n  = car.v
        x_n  = car.p
        v_n0 = car.v_max

        if car_ahead is None:
            v_n1 = 0
            x_n1 = 0
        else:
            v_n1 = car_ahead.v
            x_n1 = car_ahead.p

        # Calculate RK4 constants
        alpha_1 = traffic_obj.i_fnc(v_n, x_n, v_n0, v_n1, x_n1)
        alpha_2 = traffic_obj.i_fnc(v_n + alpha_1 * self.dt / 2, x_n, v_n0, v_n1, x_n1)
        alpha_3 = traffic_obj.i_fnc(v_n + alpha_2 * self.dt / 2, x_n, v_n0, v_n1, x_n1)
        alpha_4 = traffic_obj.i_fnc(v_n + alpha_3 * self.dt, x_n, v_n0, v_n1, x_n1)

        v_upd = (alpha_1 + 2*alpha_2 + 2*alpha_3 + alpha_4) * self.dt / 6

        # Update car positions and velocities
        car.t += self.dt
        car.v += v_upd
        car.p += car.v * self.dt + alpha_1 * self.dt**2 / 2

        car.until_driveby -= self.dt

class CAIntegrator(Integrator):
    def timestep(self, traffic_obj, car_nr):
        # Get the current car
        car = traffic_obj.cars[car_nr-1]

        # Step 1
        if car.v < car.v_max:
            car.v += 1

        # Step 2
        car_ahead = traffic_obj.find_next_car(car.lane, car.p, ahead=True, same_lane=True)
        overtaking = False
        if car.lane != traffic_obj.n_lanes:
            car_behind_next_lane = traffic_obj.find_next_car(car.lane+1, car.p, ahead=False, same_lane=False)

            if car_ahead.v < car.v\
               and (car_ahead.p - car.p) % traffic_obj.road_length < car.v\
               and (car_behind_next_lane == None or car_behind_next_lane.p != car.p):

                car.lane += 1
                overtaking = True
                car_ahead = traffic_obj.find_next_car(car.lane, car.p, ahead=True, same_lane=True)

        # Step 3
        if not overtaking and car.lane != 1:
            car_ahead_previous_lane = traffic_obj.find_next_car(car.lane-1, car.p, ahead=True, same_lane=False)
            car_behind_previous_lane = traffic_obj.find_next_car(car.lane-1, car.p, ahead=False, same_lane=False)

            if car_ahead_previous_lane == None and car_behind_previous_lane == None:
                car.lane -= 1
                car_ahead = car

            elif car.v > 0\
                and car.v < (car_ahead_previous_lane.p - car.p) % traffic_obj.road_length\
                and (car_behind_previous_lane.p - car.p) % traffic_obj.road_length:

                car.lane -= 1
                car_ahead = car_ahead_previous_lane

        # Step 4
        if (car_ahead is not car) and (car_ahead.p - car.p) % traffic_obj.road_length <= car.v:
            car.v = (car_ahead.p - car.p) % traffic_obj.road_length - 1

        # Step 5
        if car.v >= 1 and traffic_obj.p >= random.random():
            car.v -= 1

        if car.v < 0:
            pass

        # Step 6
        car.p += car.v
        car.t += 1
#################################################

############# Get pos & vel vectors #############
def get_pos_vel1(road_length, n_cars_per_lane, max_speed, n_lanes, idm=True, max_start_vel=False):
    # Initialize lanes for the cars
    first_cars = np.arange(0,n_lanes) + 1
    lanes = np.linspace(first_cars, first_cars, n_cars_per_lane)
    lanes = np.concatenate((lanes[:])).astype(int)

    # Intelligent driver model (IDM) intialization
    if idm:
        # Initialize positions
        first_pos = np.zeros(n_lanes)
        last_pos  = np.full(n_lanes, road_length - road_length / n_cars_per_lane)
        start_positions = np.linspace(first_pos, last_pos, n_cars_per_lane)
        start_positions = np.concatenate(start_positions[:])

        # Initialize velocities
        if max_start_vel:
            start_velocities = np.full((n_cars_per_lane, n_lanes), max_speed, dtype=float)
            start_velocities -= np.random.rand(n_cars_per_lane, n_lanes) * 0.2 * max_speed
            start_velocities = np.concatenate(start_velocities[:])
            max_speed_car = start_velocities
        else:
            max_speed_car  = np.full((n_cars_per_lane, n_lanes), max_speed, dtype=float)
            max_speed_car -= np.random.rand(n_cars_per_lane, n_lanes) * 0.2 * max_speed
            max_speed_car  = np.concatenate(max_speed_car[:])
            start_velocities = np.zeros((n_cars_per_lane, n_lanes))
            start_velocities = np.concatenate(start_velocities[:])

    # Cellular automata (CA) initialization
    else:
        # Initialize positions
        start_positions = np.transpose(np.array(n_lanes * [range(0,n_cars_per_lane)]))
        start_positions = np.concatenate(start_positions[:])

        # Initialize velocities
        start_velocities = np.zeros((n_cars_per_lane, n_lanes))
        start_velocities = np.concatenate(start_velocities[:])
        max_speed_car = np.full((n_cars_per_lane, n_lanes), max_speed, dtype=float)
        max_speed_car = np.concatenate(max_speed_car[:])

    # Initialize car numbers
    counter_array = np.linspace(1, n_cars_per_lane * n_lanes, n_cars_per_lane * n_lanes).astype(int)

    return lanes, start_positions, start_velocities, max_speed_car, counter_array

def get_pos_vel2(road_length, n_cars, max_speed, n_lanes, idm=True, car_length=None):
    # IDM initialization
    if idm and isinstance(car_length, float):
        if n_cars * (car_length+2) > road_length:
            raise ValueError
        start_positions  = np.arange(0, n_cars * (car_length+2), car_length+2)
        start_velocities = np.zeros(n_cars)
        max_speed_car    = np.full(n_cars, max_speed)
        max_speed_car   -= np.random.rand(n_cars) * 0.2 * max_speed
    elif idm and not isinstance(car_length, float):
        raise TypeError

    # CA initialization
    else:
        start_positions  = np.arange(0, n_cars)
        start_velocities = np.zeros(n_cars)
        max_speed_car    = np.full(n_cars, max_speed)

    lanes         = np.full(n_cars, 1)
    counter_array = np.arange(1, n_cars+1)

    return lanes, start_positions, start_velocities, max_speed_car, counter_array
#################################################

############## Params without XML ###############
def get_params_no_xml():
    RL = 50; CL = 5; MS = 3; LN = 2
    a = 0.73; delta = 4; s0 = 2; T = 1.5; b = 1.67; d = 5; gamma_max = 7; gamma_min = 1; gamma_safe = 4; overtake_time = 2
    p = 0.3

    gen_params = [RL, CL, MS, LN]
    idm_params = [a, delta, s0, T, b, d, gamma_max, gamma_min, gamma_safe, overtake_time]
    ca_params  = [p]

    return gen_params, idm_params, ca_params
#################################################

################# Get XML data ##################
def get_xml_data(xml_file):
    """ Returns 3 lists with the parameters of traffic, General - IDM - CA """

    # Load xml file and handler
    try:
        with open(xml_file) as f:
            xml = xmltodict.parse(f.read())
        xml = xml['traffic']
    except NameError:
        return get_params_no_xml()

    # Get general parameters
    RL = float(xml['road_length']); CL = int(xml['n_cars']); MS = float(xml['max_speed']); LN = int(xml['lanes'])
    general_params = [RL, CL, MS, LN]

    # Get IDM parameters
    if 'idm' in xml:
        xml_idm = xml['idm']
        a = xml_idm['a']; delta = xml_idm['delta']; s0 = xml_idm['s0']
        T = xml_idm['T']; b = xml_idm['b']; d = xml_idm['d']
        gamma_max = xml_idm['gamma_max']; gamma_min = xml_idm['gamma_min']; gamma_safe = xml_idm['gamma_safe']
        overtake_time = xml_idm['overtake_time']
        idm_params = [a, delta, s0, T, b, d, gamma_max, gamma_min, gamma_safe, overtake_time]
    else:
        idm_params = []

    # Get CA parameters
    if 'ca' in xml:
        xml_ca = xml['ca']
        p = xml_ca['p']
        ca_params = [p]
    else:
        ca_params = []

    # Turn to floats instead of strings
    idm_params = [float(x) for x in idm_params]
    ca_params = [float(x) for x in ca_params]

    return general_params, idm_params, ca_params
#################################################

################ Run animation ##################
def run_animation_idm():
    xml_file = "../xmls/input.xml"
    gen_params, idm_params, ca_params = get_xml_data(xml_file)
    traffic_obj = Traffic(gen_params, idm_params, max_start_vel=False, define_cars_per_lane=True)

    t_max = 1000
    steps_per_animation = 1
    integrator = RK4Integrator()
    numFrames = int(t_max / integrator.dt / steps_per_animation)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='polar')
    ax.axis('off')
    color = []
    for car in traffic_obj.cars:
        color.append(car.cnt)

    def animate(frameNr, steps):
        for i in range(steps):
            theta, r = traffic_obj.integrate_system(integrator)
        return ax.scatter(theta, r, c=color),

    logger.debug("Initial:\n{}".format(traffic_obj))

    anim = animation.FuncAnimation(fig, animate,
        frames=numFrames, fargs=[steps_per_animation], interval=50, blit=True, repeat=False)

    plt.show()

    logger.debug("Final:\n{}".format(traffic_obj))

    return traffic_obj
#################################################

################# Run simulation ################
def run_simulation_idm(t_max=100):
    xml_file = "../xmls/input.xml"
    gen_params, idm_params, ca_params = get_xml_data(xml_file)
    traffic_obj = Traffic(gen_params, idm_params)

    logger.debug("Initial:\n{}".format(traffic_obj))

    integrator = RK4Integrator()
    for i in range(int(t_max/integrator.dt)):
        traffic_obj.integrate_system(integrator)

    logger.debug("Final:\n{}".format(traffic_obj))

    for car in traffic_obj.cars:
        plt.figure(1)
        plt.plot(car.obs.pos, car.obs.time)
    plt.show()

    return traffic_obj
#################################################

############## Run simulation CA ################
def run_simulation_ca():
    xml_file = "../xmls/input.xml"
    gen_params, idm_params, ca_params = get_xml_data(xml_file)
    traffic_obj = Traffic(gen_params, ca_params, idm=False)

    logger.debug("Initial:\n{}".format(traffic_obj))

    t_max = 1000
    integrator = CAIntegrator()

    for i in range(int(t_max)):
        traffic_obj.integrate_system(integrator)
        logger.debug("\n{}".format(traffic_obj))

    logger.debug("Final:\n{}".format(traffic_obj))

    for car in traffic_obj.cars:
        plt.figure(1)
        plt.plot(car.obs.pos, car.obs.time)
    plt.show()

    return traffic_obj
#################################################

################ Run animation CA ###############
def run_animation_ca():
    xml_file = "../xmls/input.xml"
    gen_params, idm_params, ca_params = get_xml_data(xml_file)
    traffic_obj = Traffic(gen_params, ca_params, idm=False)

    logger.debug("Initial:\n{}".format(traffic_obj))

    t_max = 1000
    integrator = CAIntegrator()
    steps_per_animation = 1
    numFrames = int(t_max / steps_per_animation)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='polar')
    ax.axis('off')
    color = []
    for car in traffic_obj.cars:
        color.append(car.cnt)

    def animate(frameNr, steps):
        for i in range(steps):
            theta, r = traffic_obj.integrate_system(integrator)
        return ax.scatter(theta, r, c=color),

    anim = animation.FuncAnimation(fig, animate,
        frames=numFrames, fargs=[steps_per_animation], interval=50, blit=True, repeat=False)

    plt.show()

    return traffic_obj
#################################################

#################### Main #######################
def main():
    pass
#################################################

################### Start #######################
if __name__ == '__main__':
    main()
