""" This module provides a helper for the co-simulation between sumo and carla .
Modified from https://github.com/carla-simulator/carla/blob/master/Co-Simulation/Sumo/sumo_integration/bridge_helper.py
"""

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import enum
import logging
import math
import random

from collections import namedtuple

from terasim.overlay import carla, traci
import traci.constants as tc

SumoActor = namedtuple('SumoActor', 'type_id vclass transform signals extent color')

# https://sumo.dlr.de/docs/Definition_of_Vehicles,_Vehicle_Types,_and_Routes.html#abstract_vehicle_class
class SumoActorClass(enum.Enum):
    """
    SumoActorClass enumerates the different sumo actor classes.
    """
    IGNORING = "ignoring"
    PRIVATE = "private"
    EMERGENCY = "emergency"
    AUTHORITY = "authority"
    ARMY = "army"
    VIP = "vip"
    PEDESTRIAN = "pedestrian"
    PASSENGER = "passenger"
    HOV = "hov"
    TAXI = "taxi"
    BUS = "bus"
    COACH = "coach"
    DELIVERY = "delivery"
    TRUCK = "truck"
    TRAILER = "trailer"
    MOTORCYCLE = "motorcycle"
    MOPED = "moped"
    BICYCLE = "bicycle"
    EVEHICLE = "evehicle"
    TRAM = "tram"
    RAIL_URBAN = "rail_urban"
    RAIL = "rail"
    RAIL_ELECTRIC = "rail_electric"
    RAIL_FAST = "rail_fast"
    SHIP = "ship"
    CUSTOM1 = "custom1"
    CUSTOM2 = "custom2"

# ==================================================================================================
# -- Bridge helper (SUMO <=> CARLA) ----------------------------------------------------------------
# ==================================================================================================

# TODO(zyxin): load from Carla and also compare it with the one generated from the python script
# See https://github.com/carla-simulator/carla/blob/master/Co-Simulation/Sumo/data/vtypes.json
VTYPES = {
    "DEFAULT_2_WHEELED_VEHICLE": {
        "vClass": "motorcycle"
    },
    "DEFAULT_WHEELED_VEHICLE": {
        "vClass": "passenger"
    },
    "carla_blueprints": {
        "vehicle.audi.a2": {
            "vClass": "passenger"
        },
        "vehicle.audi.tt": {
            "vClass": "passenger"
        },
        "vehicle.bmw.grandtourer": {
            "vClass": "passenger"
        },
        "vehicle.chevrolet.impala": {
            "vClass": "passenger"
        },
        "vehicle.citroen.c3": {
            "vClass": "passenger"
        },
        "vehicle.jeep.wrangler_rubicon": {
            "vClass": "passenger"
        },
        "vehicle.lincoln.mkz_2017": {
            "vClass": "passenger"
        },
        "vehicle.mercedes.coupe": {
            "vClass": "passenger"
        },
        "vehicle.mini.cooper_s": {
            "vClass": "passenger"
        },
        "vehicle.ford.mustang": {
            "vClass": "passenger"
        },
        "vehicle.nissan.micra": {
            "vClass": "passenger"
        },
        "vehicle.nissan.patrol": {
            "vClass": "passenger"
        },
        "vehicle.seat.leon": {
            "vClass": "passenger"
        },
        "vehicle.volkswagen.t2": {
            "vClass": "passenger",
            "guiShape": "passenger/van"
        },
        "vehicle.dodge.charger_police": {
            "vClass": "authority",
            "guiShape": "police"
        },
        "vehicle.micro.microlino": {
            "vClass": "evehicle"
        },
        "vehicle.toyota.prius": {
            "vClass": "evehicle"
        },
        "vehicle.tesla.cybertruck": {
            "vClass": "evehicle"
        },
        "vehicle.tesla.model3": {
            "vClass": "evehicle"
        },
        "vehicle.audi.etron": {
            "vClass": "evehicle"
        },
        "vehicle.carlamotors.carlacola": {
            "vClass": "truck",
            "guiShape": "truck"
        },
        "vehicle.yamaha.yzf": {
            "vClass": "motorcycle"
        },
        "vehicle.harley-davidson.low_rider": {
            "vClass": "motorcycle"
        },
        "vehicle.kawasaki.ninja": {
            "vClass": "motorcycle"
        },
        "vehicle.gazelle.omafiets": {
            "vClass": "bicycle"
        },
        "vehicle.diamondback.century": {
            "vClass": "bicycle"
        },
        "vehicle.bh.crossbike": {
            "vClass": "bicycle"
        }
    }
}


class BridgeHelper(object):
    """
    BridgeHelper provides methos to ease the co-simulation between sumo and carla.
    """

    blueprint_library = [] # to be fetched from carla
    offset = (0, 0) # to be fetched from sumo

    @staticmethod
    def get_carla_transform(in_sumo_transform, extent):
        """
        Returns carla transform based on sumo transform.
        """
        offset = BridgeHelper.offset
        in_location = in_sumo_transform.location
        in_rotation = in_sumo_transform.rotation

        # From front-center-bumper to center (sumo reference system).
        # (http://sumo.sourceforge.net/userdoc/Purgatory/Vehicle_Values.html#angle)
        yaw = -1 * in_rotation.yaw + 90
        pitch = in_rotation.pitch
        out_location = (in_location.x - math.cos(math.radians(yaw)) * extent.x,
                        in_location.y - math.sin(math.radians(yaw)) * extent.x,
                        in_location.z - math.sin(math.radians(pitch)) * extent.x)
        out_rotation = (in_rotation.pitch, in_rotation.yaw, in_rotation.roll)

        # Applying offset sumo-carla net.
        out_location = (out_location[0] - offset[0], out_location[1] - offset[1], out_location[2])

        # Transform to carla reference system (left-handed system).
        out_transform = carla.Transform(
            carla.Location(out_location[0], -out_location[1], out_location[2]),
            carla.Rotation(out_rotation[0], out_rotation[1] - 90, out_rotation[2]))

        return out_transform
    
    @staticmethod
    def get_sumo_actor(sumo_id):
        type_id = traci.vehicle.getTypeID(sumo_id)
        vclass = SumoActorClass(traci.vehicle.getVehicleClass(sumo_id))
        color = traci.vehicle.getColor(sumo_id)

        length = traci.vehicle.getLength(sumo_id)
        width = traci.vehicle.getWidth(sumo_id)
        height = traci.vehicle.getHeight(sumo_id)

        location = list(traci.vehicle.getPosition3D(sumo_id))
        rotation = [traci.vehicle.getSlope(sumo_id), traci.vehicle.getAngle(sumo_id), 0.0]
        transform = carla.Transform(carla.Location(location[0], location[1], location[2]),
                                    carla.Rotation(rotation[0], rotation[1], rotation[2]))

        signals = traci.vehicle.getSignals(sumo_id)
        extent = carla.Vector3D(length / 2.0, width / 2.0, height / 2.0)
        
        return SumoActor(type_id, vclass, transform, signals, extent, color)

    @staticmethod
    def get_sumo_actor_from_subscription(sub):
        type_id = sub[tc.VAR_TYPE]
        vclass = SumoActorClass(sub[tc.VAR_VEHICLECLASS])
        color = sub[traci.constants.VAR_COLOR]

        length = sub[traci.constants.VAR_LENGTH]
        width = sub[traci.constants.VAR_WIDTH]
        height = sub[traci.constants.VAR_HEIGHT]

        location = list(sub[traci.constants.VAR_POSITION3D])
        rotation = [sub[traci.constants.VAR_SLOPE], sub[traci.constants.VAR_ANGLE], 0.0]
        transform = carla.Transform(carla.Location(location[0], location[1], location[2]),
                                    carla.Rotation(rotation[0], rotation[1], rotation[2]))

        signals = sub[traci.constants.VAR_SIGNALS]
        extent = carla.Vector3D(length / 2.0, width / 2.0, height / 2.0)

        return SumoActor(type_id, vclass, transform, signals, extent, color)

    @staticmethod
    def get_sumo_transform(in_carla_transform, extent):
        """
        Returns sumo transform based on carla transform.
        """
        offset = BridgeHelper.offset
        in_location = in_carla_transform.location
        in_rotation = in_carla_transform.rotation

        # From center to front-center-bumper (carla reference system).
        yaw = -1 * in_rotation.yaw
        pitch = in_rotation.pitch
        out_location = (in_location.x + math.cos(math.radians(yaw)) * extent.x,
                        in_location.y - math.sin(math.radians(yaw)) * extent.x,
                        in_location.z - math.sin(math.radians(pitch)) * extent.x)
        out_rotation = (in_rotation.pitch, in_rotation.yaw, in_rotation.roll)

        # Applying offset carla-sumo net
        out_location = (out_location[0] + offset[0], out_location[1] - offset[1], out_location[2])

        # Transform to sumo reference system.
        out_transform = carla.Transform(
            carla.Location(out_location[0], -out_location[1], out_location[2]),
            carla.Rotation(out_rotation[0], out_rotation[1] + 90, out_rotation[2]))

        return out_transform

    @staticmethod
    def _get_recommended_carla_blueprint(sumo_actor):
        """
        Returns an appropriate blueprint based on the given sumo actor.
        """
        vclass = sumo_actor.vclass.value

        blueprints = []
        for blueprint in BridgeHelper.blueprint_library:
            if blueprint.id in VTYPES and VTYPES[blueprint.id]['vClass'] == vclass:
                blueprints.append(blueprint)

        if not blueprints:
            return None

        return random.choice(blueprints)

    @staticmethod
    def get_carla_blueprint(sumo_actor: SumoActor, sync_color=False):
        """
        Returns an appropriate blueprint based on the received sumo actor.
        """
        blueprint_library = BridgeHelper.blueprint_library
        type_id = sumo_actor.type_id

        if type_id in [bp.id for bp in blueprint_library]:
            blueprint = blueprint_library.filter(type_id)[0]
            logging.debug('[BridgeHelper] sumo vtype %s found in carla blueprints', type_id)
        else:
            blueprint = BridgeHelper._get_recommended_carla_blueprint(sumo_actor)
            if blueprint is not None:
                logging.warning(
                    'sumo vtype %s not found in carla. The following blueprint will be used: %s',
                    type_id, blueprint.id)
            else:
                logging.error('sumo vtype %s not supported. No vehicle will be spawned in carla',
                              type_id)
                return None

        if blueprint.has_attribute('color'):
            if sync_color:
                color = "{},{},{}".format(sumo_actor.color[0], sumo_actor.color[1],
                                          sumo_actor.color[2])
            else:
                color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)

        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)

        blueprint.set_attribute('role_name', 'sumo_driver')

        logging.debug(
            '''[BridgeHelper] sumo vtype %s will be spawned in carla with the following attributes:
            \tblueprint: %s
            \tcolor: %s''', type_id, blueprint.id,
            sumo_actor.color if blueprint.has_attribute('color') else (-1, -1, -1))

        return blueprint
