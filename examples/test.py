# coding=utf-8
import sys 
from pathlib import Path
sys.path.append('/home/chris/github/TeraSim/')
import traci  # noqa

traci.start(["sumo-gui", "-c", "examples/maps/mcity/mcity_medium_traffic.sumocfg"], port=7911)
while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
    print(traci.trafficlight.getRedYellowGreenState("NODE_11"))
traci.close()
sys.exit()