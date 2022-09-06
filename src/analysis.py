# Henry Jacobson, Simulation and Modeling, KTH 2020

################## Imports ##################
from ML import *
from time import time
from itertools import cycle
import sys
#############################################

############ Calculate flowrate #############
def flowrate_calculation(traffic_obj):
    """ Returns a list of the flowrate at each time step of traffic_obj. """
    velocities = np.zeros(shape=len(traffic_obj.cars[0].obs.vel))
    for car in traffic_obj.cars:
        velocities += car.obs.vel
    flowrates = velocities / traffic_obj.road_length
    times = traffic_obj.cars[0].obs.time

    return flowrates, times
##############################################

############ Get final flowrate ##############
def get_final_flowrate(flowrates):
    return np.mean(flowrates[len(flowrates)//2::])
##############################################

########## Plot flowrate agains time #########
def plot_flowrate(traffic_obj, fig_nr=1):
    flowrates, times = flowrate_calculation(traffic_obj)

    plt.figure()
    plt.plot(times, flowrates)
    plt.xlabel("Time")
    plt.ylabel("Flow rate")
    plt.legend(("{} lanes".format(traffic_obj.n_lanes),),prop={'size':15})
    plt.title("Flow rate vs time")
    plt.savefig("../img/flowrate_plot.png")
    plt.show()

    return flowrates, times
##############################################

##### Plot flowrates for different lanes #####
def flowrate_lanes_plot(idm=True, mean=False):
    xml_file = "../xmls/input.xml"
    gen_params, idm_params, ca_params = get_xml_data(xml_file)
    cars_per_lane = [30, 15, 10]; n_lanes = [1,2,3]
    if idm:
        integrator = RK4Integrator()
        t_max = 100
    else:
        integrator = CAIntegrator()
        if mean:
            t_max = 10000
        else:
            t_max = 1000

    flowrates = [0 for x in range(3)]
    for i in range(len(cars_per_lane)):
        gen_params[1] = cars_per_lane[i]; gen_params[3] = n_lanes[i]
        if idm:
            traffic_obj = Traffic(gen_params, idm_params)
            dt = integrator.dt
        else:
            traffic_obj = Traffic(gen_params, ca_params, idm=False)
            dt = 1
        for j in range(int(t_max / dt)):
            traffic_obj.integrate_system(integrator)
        flowrates[i], times = flowrate_calculation(traffic_obj)

    plt.figure(3)
    if mean:
        for i in range(3):
            flowrates_mean, times_mean = get_mean(flowrates[i], times)
            plt.plot(times_mean, flowrates_mean)
    else:
        for i in range(3):
            plt.plot(times, flowrates[i])
    plt.xlabel("Time")
    plt.ylabel("Flowrate")
    plt.legend(("1 lane", "2 lanes", "3 lanes"), prop={'size':15})
    if idm:
        plt.title(r"Flowrate vs # of lanes, IDM, $\rho=0.03$")
        plt.savefig("../img/flowrates_lanes_IDM.png")
    else:
        if mean:
            plt.title(r"Flowrate vs # of lanes, CA, $\rho=0.03$, mean of flowrate")
            plt.savefig("../img/flowrates_lanes_CA_mean.png")
        else:
            plt.title(r"Flowrate vs # of lanes, CA, $\rho=0.03$")
            plt.savefig("../img/flowrates_lanes_CA.png")
    plt.show()
##############################################

####### Plot flowrates for idm and ca ########
def flowrate_model_plot():
    xml_file = "../xmls/input.xml"
    gen_params, idm_params, ca_params = get_xml_data(xml_file)

    idm_integrator = RK4Integrator()
    ca_integrator  = CAIntegrator()

    t_max = 1000

    p_s = [0.2, 0.3, 0.4, 0.5, 0.6]

    plt.figure(4)
    for p in p_s:
        ca_params[0] = p
        traffic_ca = Traffic(gen_params, ca_params, idm=False)
        logger.info("\n{}".format(traffic_ca))
        for i in range(int(t_max)):
            traffic_ca.integrate_system(ca_integrator)
        ca_flowrate, ca_times = flowrate_calculation(traffic_ca)
        plt.plot(ca_times, ca_flowrate)
    traffic_idm = Traffic(gen_params, idm_params, max_start_vel=False)
    logger.info("\n{}".format(traffic_idm))
    for i in range(int(t_max / idm_integrator.dt)):
        traffic_idm.integrate_system(idm_integrator)

    ca_flowrate, ca_times   = flowrate_calculation(traffic_ca)
    idm_flowrate, idm_times = flowrate_calculation(traffic_idm)

    plt.plot(idm_times, idm_flowrate, linewidth=5, color='red')
    plt.xlabel("Time", fontsize=12)
    plt.ylabel("Flowrate", fontsize=12)
    plt.legend(("CA, p=0.2", "CA, p=0.3", "CA, p=0.4", "CA, p=0.5", "CA, p=0.6", "IDM"), prop={'size':15})
    plt.title("Difference between flowrate of IDM and CA", fontsize=15)
    plt.savefig("../img/flowrate_idm_v_ca.png")
    plt.show()
##############################################

########### Get mean of flowrate #############
def get_mean(flowrates, times):
    cnt = 0
    mean_start = 500
    flowrates_mean = np.empty(0)
    times_mean = np.empty(0)
    for flowrate in flowrates:
        if cnt > mean_start and not cnt % 50:
            flowrates_mean = np.append(flowrates_mean, np.mean(flowrates[cnt-mean_start:cnt]))
            times_mean = np.append(times_mean, times[cnt])
        cnt += 1
    return flowrates_mean, times_mean
##############################################

########### Fundamental diagram ##############
def fundamental_diagram(road_length, n_points, n_lanes, idm=True, plot=False, vmax=None, density_full=False):
    xml_file = "../xmls/input.xml"
    gen_params, idm_params, ca_params = get_xml_data(xml_file)

    if idm:
        if density_full:
            cars_per_lane_list = [int(x) for x in np.linspace(n_lanes, n_lanes * road_length / (idm_params[5]+2), int(n_points/n_lanes))]
        else:
            cars_per_lane_list = [int(x) for x in np.linspace(n_lanes, road_length / (idm_params[5]+2), int(n_points/n_lanes))]
        idm_param = idm_params[5] + 2
        integrator = RK4Integrator()
        dt = integrator.dt
        t_max = 100
        gen_params[2] = 30
    else:
        if density_full:
            cars_per_lane_list = [int(x) for x in np.linspace(n_lanes, n_lanes * road_length, int(n_points/n_lanes))]
        else:
            cars_per_lane_list = [int(x) for x in np.linspace(n_lanes, road_length, int(n_points/n_lanes))]
        integrator = CAIntegrator()
        idm_param = 1
        dt = 1
        t_max = 1000

    densities = np.empty(0)
    flowrate_list = np.empty(0)
    for n_cars in cars_per_lane_list:
        gen_params[0] = road_length; gen_params[1] = int(n_cars / n_lanes)
        gen_params[3] = n_lanes

        if vmax:
            gen_params[2] = vmax

        if idm:
            traffic_obj = Traffic(gen_params, idm_params, idm=idm, max_start_vel=False)
        else:
            traffic_obj = Traffic(gen_params, ca_params, idm=idm, max_start_vel=False)
        logger.info("\n{}".format(traffic_obj))
        for i in range(int(t_max / dt)):
            traffic_obj.integrate_system(integrator)

        flowrates, times = flowrate_calculation(traffic_obj)
        flowrate_list = np.append(flowrate_list, get_final_flowrate(flowrates))
        if density_full:
            densities = np.append(densities, n_cars / (road_length * n_lanes) * idm_param)
        else:
            densities = np.append(densities, n_cars / road_length * idm_param)

    if plot:
        plt.figure(1)
        plt.plot(densities, flowrate_list, "b.")
        plt.xlabel(r"Density, $\rho$")
        plt.ylabel(r"Flowrate")
        plt.title("Flowrate vs Density")
        plt.grid()
        if idm:
            plt.savefig("../img/fundamental_diagram_IDM_{}lane.png".format(n_lanes))
        else:
            plt.savefig("../img/fundamental_diagram_CA.png")
        plt.show()

    return flowrate_list, densities
##############################################

######## Fundamental diagram vs lanes ########
def fund_diag_lanes(road_length, n_points, idm=True, vmax=None, density_full=False):
    plt.figure(2)
    max_flowrates = []
    for i in range(3):
        flowrates, densities = fundamental_diagram(road_length, n_points, i+1, idm=idm, vmax=vmax, density_full=density_full)
        plt.plot(densities, flowrates)
        max_flowrates.append(np.max(flowrates))

    logger.info("Max flowrates:\n{}".format(max_flowrates))

    if idm:
        model = "IDM"
    else:
        model = "CA"

    plt.xlabel(r"Density, $\rho$")
    plt.ylabel(r"Flowrate")
    plt.title("Fundamental diagram for different # of lanes, {}".format(model))
    plt.legend(("1 lane", "2 lanes", "3 lanes"), prop={'size':15})
    plt.grid()
    if density_full:
        plt.savefig("../img/fundamental_diagram_vs_lanes_{}_dens.png".format(model))
    else:
        plt.savefig("../img/fundamental_diagram_vs_lanes_{}.png".format(model))
    plt.show()
##############################################

######### Standard error calculation #########
def std_err_calc(road_length, n_cars, v_max, lanes, n_simulations_max, idm=True, p=None, steps=2, plot=False):
    xml_file = "../xmls/input.xml"
    gen_params, idm_params, ca_params = get_xml_data(xml_file)
    gen_params = [road_length, n_cars, v_max, lanes]

    t_max = 1000
    if not idm:
        ca_params = [p]
        integrator = CAIntegrator()
        dt = 1
        traffic_obj = Traffic(gen_params, ca_params, idm=idm)
    else:
        integrator = RK4Integrator()
        dt = integrator.dt
        traffic_obj = Traffic(gen_params, idm_params, idm=idm, max_start_vel=False)

    std_errs = np.zeros(0)
    sim_list   = np.zeros(0)
    flowrate_list = np.zeros(0)
    for N in range(2, n_simulations_max, steps):
        for i in range(steps):
            for j in range(int(t_max / dt)):
                traffic_obj.integrate_system(integrator)
            flowrates, times = flowrate_calculation(traffic_obj)
            flowrate = np.mean(flowrates[len(flowrates)//2::])
            flowrate_list = np.append(flowrate_list, flowrate)
        if len(flowrate_list) > 1:
            std_errs = np.append(std_errs, np.sqrt(np.var(flowrate_list) / (len(flowrate_list) - 1)))
            sim_list   = np.append(sim_list, N)
            logger.info("N={}, Error list: {}".format(N, std_errs))

    if plot:
        plt.figure()
        plt.plot(sim_list, std_errs)
        plt.xlabel("# of simulations")
        plt.ylabel("Standard error")
        plt.title("Standard error vs # of simulations")
        if not idm:
            plt.savefig("../img/std_err_traffic_vmax{}_p{}.png".format(v_max,p))
        else:
            plt.savefig("std_err_traffic_vmax{}.png".format(v_max))
        plt.show()

    return std_errs, sim_list
##############################################

########### Standard error vs lanes ##########
def std_err_lanes(road_length, n_cars, v_max, n_simulations_max, idm=True, p=None, steps=2):
    std_errs_list = np.empty(0); sim_list_list = np.empty(0)
    cars_multiply = [3, 1.5, 1]
    plt.figure()
    for lanes in range(1,4):
        std_errs, sim_list = std_err_calc(road_length, int(n_cars * cars_multiply[lanes-1]), v_max, lanes, n_simulations_max,\
            idm=idm, p=p, steps=steps)
        plt.plot(sim_list, std_errs)

    plt.xlabel("# of simulations")
    plt.ylabel("Standard error")
    plt.legend(("1 lane", "2 lanes", "3 lanes"), prop={'size':15})
    plt.title("Standard error for different amount of lanes, IDM")
    if idm:
        plt.title("Standard error for different amount of lanes, IDM")
        plt.savefig("../img/std_err_lanes_IDM.png")
    else:
        plt.title("Standard error for different amount of lanes, CA")
        plt.savefig("../img/std_err_lanes_CA_simmax{}.png".format(n_simulations_max))
    plt.show()
##############################################

######## Fund diag lanes vs max speeds #######
def fund_diag_lanes_vmax(road_length, n_points, idm=True):
    plt.figure(5)
    lines = ["-","--","-."]
    linecycler = cycle(lines)
    colors = ["r", "r", "r", "g", "g", "g", "b", "b", "b"]
    colorcycler = cycle(colors)
    if idm:
        vmaxs = [20, 30, 40]
    else:
        vmaxs = [2,3,5]
    for vmax in vmaxs:
        for i in range(3):
            flowrates, densities = fundamental_diagram(road_length, n_points, i+1, idm=idm, vmax=vmax)
            plt.plot(densities, flowrates, next(linecycler), color=next(colorcycler))

    if idm:
        model = "IDM"
    else:
        model = "CA"

    plt.xlabel(r"Density, $\rho$", fontsize=18)
    plt.ylabel(r"Flowrate", fontsize=18)
    plt.title("Fundamental diagram for different # of lanes and maximum velocity, {}".format(model), fontsize=21)
    plt.legend((r"1 lane, $v_{max}=2$", r"2 lanes, $v_{max}=2$", r"3 lanes, $v_{max}=2$",\
        r"1 lane, $v_{max}=3$", r"2 lanes, $v_{max}=3$", r"3 lanes, $v_{max}=3$",\
        r"1 lane, $v_{max}=5$", r"2 lanes, $v_{max}=5$", r"3 lanes, $v_{max}=5$"), prop={'size':16})
    plt.grid()
    plt.savefig("../img/fundamental_diagram_vs_lanes_{}_Vmax.png".format(model))
    plt.show()
##############################################

########## Amount of cars in lanes ###########
def n_cars_in_lanes(road_length, n_cars, n_lanes, v_max, idm=True, plot=True):
    xml_file = "../xmls/input.xml"
    gen_params, idm_params, ca_params = get_xml_data(xml_file)
    gen_params[0] = road_length; gen_params[1] = n_cars
    gen_params[2] = v_max;       gen_params[3] = n_lanes

    if idm:
        integrator = RK4Integrator()
        t_max = 100
        dt = integrator.dt
        traffic_obj = Traffic(gen_params, idm_params, max_start_vel=False)
        model="IDM"
    else:
        integrator = CAIntegrator()
        t_max = 1000
        dt = 1
        traffic_obj = Traffic(gen_params, ca_params, idm=False)
        model="CA"

    logger.info("\n{}".format(traffic_obj))
    for i in range(int(t_max / dt)):
        traffic_obj.integrate_system(integrator)

    cars_in_lane = np.zeros((n_lanes, int(t_max/dt)+1))
    for car in traffic_obj.cars:
        cnt = 0
        for time in car.obs.time:
            cars_in_lane[car.obs.lane[cnt]-1, cnt] += 1
            cnt += 1
    for elem in cars_in_lane.T:
        if np.sum(elem) != n_cars * n_lanes:
            raise Exception

    if plot:
        linestyle = ["r", "b", "g"]
        plt.figure()
        for lane in range(n_lanes):
            plt.plot(traffic_obj.cars[0].obs.time, cars_in_lane[lane], linestyle[lane])
        plt.xlabel("Time")
        plt.ylabel("Number of cars")
        plt.legend(("Lane 1", "Lane 2", "Lane 3"), prop={'size':15})
        plt.title("Amount of cars in each lane at each time, {}".format(model))
        if idm:
            plt.savefig("../img/n_cars_in_lanes_IDM.png")
        else:
            plt.savefig("../img/n_cars_in_lanes_CA.png")
        plt.show()

    return cars_in_lane, traffic_obj
##############################################

######### Density vs cars_in_lanes ###########
def density_cars_lanes(road_length, n_points, n_lanes, idm=True, vmax=None, density_full=False):
    xml_file = "../xmls/input.xml"
    gen_params, idm_params, ca_params = get_xml_data(xml_file)

    if idm:
        if density_full:
            cars_per_lane_list = [int(x) for x in np.linspace(n_lanes, n_lanes * road_length / (idm_params[5]+2), int(n_points/n_lanes))]
        else:
            cars_per_lane_list = [int(x) for x in np.linspace(n_lanes, road_length / (idm_params[5]+2), int(n_points/n_lanes))]
        idm_param = idm_params[5] + 2
        integrator = RK4Integrator()
        dt = integrator.dt
        t_max = 100
        gen_params[2] = 30
        model="IDM"
    else:
        if density_full:
            cars_per_lane_list = [int(x) for x in np.linspace(n_lanes, n_lanes * road_length, int(n_points/n_lanes))]
        else:
            cars_per_lane_list = [int(x) for x in np.linspace(n_lanes, road_length, int(n_points/n_lanes))]
        integrator = CAIntegrator()
        idm_param = 1
        dt = 1
        t_max = 100
        model="CA"

    if vmax:
        gen_params[2] = vmax

    density_car_in_lane = np.zeros((n_lanes,len(cars_per_lane_list)))
    densities = np.zeros(len(cars_per_lane_list))
    cnt = 0
    for n_cars in cars_per_lane_list:
        cars_in_lane, traffic_obj = n_cars_in_lanes(road_length, int(n_cars/n_lanes), n_lanes, v_max=gen_params[2], idm=idm, plot=False)
        final_cars_in_lane = np.mean(cars_in_lane[:, cars_in_lane.shape[1]//2::], axis=1)
        density_car_in_lane.T[cnt] = final_cars_in_lane / (n_cars)
        if density_full:
            densities[cnt] = n_cars / (road_length * n_lanes) * idm_param
        else:
            densities[cnt] = n_cars / (road_length) * idm_param
        cnt += 1

    linestyle = ["r", "b", "g"]
    plt.figure()
    for lane in range(n_lanes):
        plt.plot(densities, density_car_in_lane[lane], linestyle[lane])
    plt.plot([0,1],[1/n_lanes,1/n_lanes], "--y")
    plt.xlabel(r"Density $\rho$")
    plt.ylabel("Percentage of cars in lane")
    plt.legend(("Lane 1", "Lane 2", "Lane 3"), prop={'size':15})
    plt.title("Cars in each lane vs. the density of the road, {}".format(model))
    if idm:
        plt.savefig("../img/cars_each_lane_density_IDM.png")
    else:
        plt.savefig("../img/cars_each_lane_density_CA.png")
    plt.show()
##############################################

################### Main #####################
def main():
    if len(sys.argv) < 2:
        exit("Specify analysis to be done.")
    options = {
        "idm_flowrate":{"fnc":plot_flowrate,"args":run_simulation_idm,"kwargs":{}},
        "ca_flowrate":{"fnc":plot_flowrate,"args":run_simulation_ca,"kwargs":{}},
        "idm_flowrates_lanes":{"fnc":flowrate_lanes_plot,"args":[],"kwargs":{"idm":True}},
        "ca_flowrates_lanes":{"fnc":flowrate_lanes_plot,"args":[],"kwargs":{"idm":False}},
        "ca_flowrates_lanes_mean":{"fnc":flowrate_lanes_plot,"args":[],"kwargs":{"idm":False,"mean":True}},
        "idm_fundamental":{"fnc":fundamental_diagram,"args":[1000, 100, 1],"kwargs":{"idm":True,"plot":True}},
        "ca_fundamental":{"fnc":fundamental_diagram,"args":[100,50,2],"kwargs":{"idm":False,"plot":True}},
        "ca_fundamental_lanes":{"fnc":fund_diag_lanes,"args":[100, 50],"kwargs":{"idm":False,"vmax":2}},
        "idm_fundamental_lanes":{"fnc":fund_diag_lanes,"args":[1000,100],"kwargs":{"idm":True,"vmax":30}},
        "flowrate_diff":{"fnc":flowrate_model_plot,"args":[None],"kwargs":{}},
        "ca_stderr":{"fnc":std_err_calc,"args":[100,15,2,2,100],"kwargs":{"idm":False,"p":0.5,"plot":True}},
        "ca_stderr_lanes":{"fnc":std_err_lanes,"args":[50,10,2,51],"kwargs":{"idm":False,"p":0.3,"steps":1}},
        "idm_stderr_lanes":{"fnc":std_err_lanes,"args":[500,10,30,100],"kwargs":{"idm":True,"steps":2}},
        "ca_fundamental_diff_vmax":{"fnc":fund_diag_lanes_vmax,"args":[50,25],"kwargs":{"idm":False}},
        "ca_fundamental_road_size":{"fnc":fund_diag_lanes,"args":[100,50],"kwargs":{"idm":False,"vmax":2,"density_full":True}},
        "idm_fundamental_road_size":{"fnc":fund_diag_lanes,"args":[500,100],"kwargs":{"idm":True,"vmax":30,"density_full":True}},
        "ca_n_cars_in_lane":{"fnc":n_cars_in_lanes,"args":[100,12,3,2],"kwargs":{"idm":False}},
        "idm_n_cars_in_lane":{"fnc":n_cars_in_lanes,"args":[1000,10,3,30],"kwargs":{"idm":True}},
        "ca_density_cars_lanes":{"fnc":density_cars_lanes,"args":[100,100,3],"kwargs":{"idm":False,"vmax":2,"density_full":True}},
        "ca_density_cars_lanes":{"fnc":density_cars_lanes,"args":[500,100,3],"kwargs":{"idm":True,"vmax":30,"density_full":True}},
    }
    if sys.argv[1] not in options:
        print("Not a valid option, implemented analysis options:")
        for key in options.keys():
            print("\t",key)
        exit()

    if sys.argv[1] in ("ca_flowrate","idm_flowrate"):
        options[sys.argv[1]]["fnc"](options[sys.argv[1]]["args"]())
    else:
        options[sys.argv[1]]["fnc"](*options[sys.argv[1]]["args"],**options[sys.argv[1]]["kwargs"])
##############################################

if __name__ == '__main__':
    main()
