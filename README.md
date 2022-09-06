# TrafficSimulation
![Fundamental diagram IDM](/img/fundmental_diagram_vs_lanes_IDM_dens.png)
Final Project in course Simulation and Modelling during Winter 2020/2021

This repo contains the source code, results and report of a traffic simulation on a circular highway consisting of different amount of lanes.
The main goal of the project was to model and simulate some real world phenomenon, as well as gather results and analyze them.

## Source code
The main part of source code is found in `src/ML.py`, where definition of classes for a Traffic class, a Car class and some integrator classes are.
The analysis using this source code is found in `src/analysis.py` which accept a command line argument that defines what kind of analysis to be done.
xml-files are used to determine the parameters of the model, found in `xmls/input.xml`.
