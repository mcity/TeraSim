import os
import json
import redis
import sumolib
import lxml.etree as ET

from terasim.overlay import traci
from terasim.envs.base import BaseEnv
from terasim.simulator import Simulator


class TeraSimCoSimPlugin(BaseEnv):

    def __init__(self, vehicle_factory, info_extractor):
        self.net = None
        self._routes = set()
        self.terasim_controlled_vehicle_ids = set()
        self.carla2sumo_ids = set()

        self.ctx = None
        self.simulator = None
        self.redis_client = None

        super().__init__(vehicle_factory, info_extractor)

    def on_start(self, simulator: Simulator, ctx):
        redis_host = os.environ.get("TERASIM_REDIS_HOST", 'localhost')
        redis_port = os.environ.get("TERASIM_REDIS_PORT", 6379)
        redis_password = os.environ.get("TERASIM_REDIS_PASSWORD", "")

        self.redis_client = redis.Redis(host=redis_host, port=redis_port, db=0, password=redis_password)
        print("Redis server connected")
        
        self.net = self._get_sumo_net(self.simulator.sumo_config_file_path)

    def on_step(self, simulator: Simulator, ctx):
        self.sync_traffic_light()

        self.sync_sumo_vehicle_to_carla()
        self.sync_carla_vehicle_to_sumo()

        # update sumo controlled vehicles to global context
        self.ctx["terasim_controlled_vehicle_ids"] = list(self.terasim_controlled_vehicle_ids)
        
        return True

    def on_stop(self, simulator: Simulator, ctx):
        pass

    def inject(self, simulator: Simulator, ctx):
        self.ctx = ctx
        self.simulator = simulator

        simulator.start_pipeline.hook("cosim_start", self.on_start, priority=-100)
        simulator.step_pipeline.hook("cosim_step", self.on_step, priority=-100)
        simulator.stop_pipeline.hook("cosim_stop", self.on_stop, priority=-100)

    def sync_traffic_light(self):
        cosim_traffic_light_state = self.redis_client.get("cosim_traffic_light_state")
        if cosim_traffic_light_state is not None:
            signal_information = json.loads(cosim_traffic_light_state)
            for signal_id in signal_information:
                traci.trafficlight.setRedYellowGreenState(signal_id, signal_information[signal_id])

    def sync_sumo_vehicle_to_carla(self):
        ''' sync sumo controlled vehicle to carla '''   
        veh_list = self.simulator.get_vehID_list()

        self.terasim_controlled_vehicle_ids = {
            vehID for vehID in veh_list if 'BV' in vehID or 'CAV' in vehID
        }
        self.terasim_controlled_vehicle_ids = set(self.terasim_controlled_vehicle_ids)

        cosim_terasim_vehicle_info = {}

        for vehID in self.terasim_controlled_vehicle_ids:
            pos = traci.vehicle.getPosition3D(vehID)
            slope = traci.vehicle.getSlope(vehID)
            angle = traci.vehicle.getAngle(vehID)

            actor = {
                "location": { "x": pos[0], "y": pos[1], "z": pos[2]},
                "rotation": { "x": slope, "y": angle, "z" : 0.0},
                "extent": {
                    "x": traci.vehicle.getLength(vehID) / 2.0,
                    "y": traci.vehicle.getWidth(vehID) / 2.0,
                    "z": traci.vehicle.getHeight(vehID) / 2.0
                },
                "lights": traci.vehicle.getSignals(vehID),
                "speed": traci.vehicle.getSpeed(vehID)
            }

            cosim_terasim_vehicle_info[vehID] = actor

        cosim_terasim_vehicle_info_json = json.dumps(cosim_terasim_vehicle_info)
        self.redis_client.set('cosim_terasim_vehicle_info', cosim_terasim_vehicle_info_json)

    def sync_carla_vehicle_to_sumo(self):
        ''' update carla controlled vehicle in sumo '''
        cosim_thirdpartysim_vehicle_info = self.redis_client.get('cosim_thirdpartysim_vehicle_info')
        if cosim_thirdpartysim_vehicle_info is not None:
            cosim_thirdpartysim_vehicle_info = json.loads(cosim_thirdpartysim_vehicle_info)

            for vehID in cosim_thirdpartysim_vehicle_info:
                if vehID not in self.carla2sumo_ids:
                    try:
                        vclass = traci.vehicletype.getVehicleClass('IDM_waymo_motion')
                        if vclass not in self._routes:
                            print('Creating route for %s vehicle class', vclass)
                            allowed_edges = [e for e in self.net.getEdges() if e.allows(vclass)]
                            if allowed_edges:
                                traci.route.add("carla_route_{}".format(vclass), [allowed_edges[0].getID()])
                                self._routes.add(vclass)
                            else:
                                print('Could not found a route for %s. No vehicle will be spawned in sumo', 'IDM_waymo_motion')

                        self.add_vehicle(vehID, 'carla_route_{}'.format(vclass), lane_id=None, speed=0)
                        traci.vehicle.setColor(vehID, (150, 255, 255, 255))
                        self.carla2sumo_ids.add(vehID)
                        print('Adding vehicle to sumo successful', vehID)

                    except traci.exceptions.TraCIException as error:
                        print('Spawn sumo actor failed: %s', error)
                else:
                    loc_x = cosim_thirdpartysim_vehicle_info[vehID]['location']['x']
                    loc_y = cosim_thirdpartysim_vehicle_info[vehID]['location']['y']
                    yaw = cosim_thirdpartysim_vehicle_info[vehID]['rotation']['z']

                    traci.vehicle.moveToXY(vehID, "", 0, loc_x, loc_y, angle=yaw, keepRoute=0)

            # remove vehicles that are not in cosim_thirdpartysim_vehicle_info
            to_remove = [vehID for vehID in self.carla2sumo_ids if vehID not in cosim_thirdpartysim_vehicle_info]
            self._remove_vehicle_from_env(to_remove)
            self.carla2sumo_ids -= set(to_remove)
        else:
            self._remove_vehicle_from_env(list(self.carla2sumo_ids))
            self.carla2sumo_ids.clear()

    def _get_sumo_net(self, cfg_file):
        """
        Returns sumo net.

        This method reads the sumo configuration file and retrieve the sumo net filename to create the
        net.
        """
        cfg_file = os.path.join(os.getcwd(), cfg_file)

        tree = ET.parse(cfg_file)
        tag = tree.find('//net-file')
        if tag is None:
            return None

        net_file = os.path.join(os.path.dirname(cfg_file), tag.get('value'))
        print('Reading net file: %s', net_file)

        sumo_net = sumolib.net.readNet(net_file)
        return sumo_net
