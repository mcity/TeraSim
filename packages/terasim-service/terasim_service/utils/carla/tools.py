import carla
import logging
import math
import pyproj
import utm


ZONE_NUMBER = 17
ZONE_LETTER = "T"
GNSS_ORIGIN = [42.3005934157, -83.699283188811]

TLS_NODES = {
    "NODE_11": [(83,), (92,), None, (88,), (89,), (86,), None, (84,)],
    "NODE_12": [(78,), (82,), (81,), (80,)],
    "NODE_17": [(61,), (62,), (67,), (68,), (65,), (66,), None, (63,), (64,)],
    "NODE_18": [(69, 70), None, (75,), None, (73, 74), None, (71,), None],
    "NODE_23": [(110,), None, (112,), (107,)],
    "NODE_24": [(47,), None, (48,), (52,)],
}


def create_vehicle_blueprint(world):
    blueprint_library = world.get_blueprint_library()

    car_keywords = [
        "vehicle.lincoln.mkz_2020",
        # "vehicle.audi",
        # "vehicle.bmw",
        # "vehicle.chevrolet",
        # "vehicle.citroen",
        # "vehicle.dodge",
        # "vehicle.mercedes",
        # "vehicle.nissan",
        # "vehicle.seat",
        # "vehicle.toyota",
        # "vehicle.tesla",
        # "vehicle.volkswagen",
    ]

    vehicle_blueprints = [
        bp
        for bp in blueprint_library.filter("vehicle.*")
        if any(keyword in bp.id for keyword in car_keywords)
    ]
    return vehicle_blueprints


def create_bike_blueprint(world):
    blueprint_library = world.get_blueprint_library()

    bike_keywords = [
        "vehicle.gazelle.omafiets",
        "vehicle.diamondback.century",
        "vehicle.bh.crossbike",
    ]

    bike_blueprints = [
        bp
        for bp in blueprint_library.filter("vehicle.*")
        if any(keyword in bp.id for keyword in bike_keywords)
    ]

    return bike_blueprints


def create_pedestrian_blueprint(world):
    blueprint_library = world.get_blueprint_library()

    pedestrian_blueprints = blueprint_library.filter("walker.pedestrian.*")

    return pedestrian_blueprints


def create_motor_blueprint(world):
    blueprint_library = world.get_blueprint_library()
    motor_keywords = [
        "vehicle.yamaha.yzf",
        "vehicle.vespa.zx125",
        "vehicle.kawasaki.ninja",
        "vehicle.harley-davidson.low_rider",
    ]
    motor_blueprints = [
        bp
        for bp in blueprint_library.filter("vehicle.*")
        if any(keyword in bp.id for keyword in motor_keywords)
    ]
    return motor_blueprints


def create_bikeandmotor_blueprint(world):
    blueprint_library = world.get_blueprint_library()
    bikeandmotor_keywords = [
        "vehicle.gazelle.omafiets",
        "vehicle.diamondback.century",
        "vehicle.bh.crossbike",
        "vehicle.yamaha.yzf",
        "vehicle.vespa.zx125",
        "vehicle.kawasaki.ninja",
        "vehicle.harley-davidson.low_rider",
    ]
    bikeandmotor_blueprints = [
        bp
        for bp in blueprint_library.filter("vehicle.*")
        if any(keyword in bp.id for keyword in bikeandmotor_keywords)
    ]
    return bikeandmotor_blueprints


def create_construction_zone_blueprint(world):
    blueprint_library = world.get_blueprint_library()
    construction_zone_blueprint = blueprint_library.find(
        "static.prop.trafficcone01"
    )
    return construction_zone_blueprint


def create_police_car_blueprint(world):
    blueprint_library = world.get_blueprint_library()
    police_car_keywords = [
        "vehicle.dodge.charger_police",
        "vehicle.dodge.charger_police_2020",
    ]
    police_car_blueprints = [
        bp
        for bp in blueprint_library.filter("vehicle.*")
        if any(keyword in bp.id for keyword in police_car_keywords)
    ]
    return police_car_blueprints


def isVehicle(actorID):
    return "BV" in actorID or "AV" in actorID or "POV" in actorID or "VUT" in actorID


def isPedestrian(actorID):
    return "VRU" in actorID


def spawn_actor(client, blueprint, transform):
    """
    Spawns a new actor.

        :param blueprint: blueprint of the actor to be spawned.
        :param transform: transform where the actor will be spawned.
        :return: actor id if the actor is successfully spawned. Otherwise, INVALID_carla_id.
    """

    batch = [
        carla.command.SpawnActor(blueprint, transform).then(
            carla.command.SetSimulatePhysics(carla.command.FutureActor, False)
        )
    ]
    response = client.apply_batch_sync(batch, True)[0]
    if response.error:
        logging.error("Spawn carla actor failed. %s", response.error)
        return -1

    return response.actor_id


def destroy_all_actors(world):
    carla_actors = (
        list(world.get_actors().filter("vehicle.*"))
        + list(world.get_actors().filter("walker.pedestrian.*"))
        + list(world.get_actors().filter("static.prop.constructioncone"))
    )

    for actor in carla_actors:
        actor.destroy()


def get_actor_id_from_attribute(world, attribute):
    actor_list = world.get_actors()
    for actor in actor_list:
        if actor.attributes.get("role_name") == attribute:
            return True, actor.id

    return False, -1


def get_z_offset(world, start_location, end_location, previous_state=None):
    raycast_result = world.cast_ray(start_location, end_location)
    if not raycast_result:
        print("Ray did not hit the ground.")
        return 0

    if previous_state is None:
        # If previous_state is None, just return the minimum `z` value in raycast results.
        height = min((item.location.z for item in raycast_result), default=0)
    else:
        # Find the height closest to previous_state.
        height = min(
            (
                item.location.z
                for item in raycast_result
                if item.label == carla.CityObjectLabel.Roads
            ),
            default=raycast_result[0].location.z,
            key=lambda z: abs(previous_state - z),
        )
    return height


# Define the function to draw text in the simulation
def draw_text(world, location, text, color=(255, 0, 0), life_time=0.05):
    debug = world.debug
    debug.draw_string(
        location,
        text,
        draw_shadow=False,
        color=carla.Color(r=color[0], g=color[1], b=color[2]),
        life_time=life_time,
        persistent_lines=True,
    )


# Define the function to draw text in the simulation
def draw_point(world, location, size=0.1, color=(255, 0, 0), life_time=0.05):
    debug = world.debug
    debug.draw_point(
        location,
        size,
        color=carla.Color(r=color[0], g=color[1], b=color[2]),
        life_time=life_time,
    )


def update_spectator_camera(self, vehicle_transform, follow_distance=10):
    spectator = self.world.get_spectator()
    vehicle_location = vehicle_transform.location
    vehicle_rotation = vehicle_transform.rotation

    # Calculate the position of the camera relative to the vehicle
    camera_location = carla.Location(
        x=vehicle_location.x
        - follow_distance * math.cos(math.radians(vehicle_rotation.yaw)),
        y=vehicle_location.y
        - follow_distance * math.sin(math.radians(vehicle_rotation.yaw)),
        z=vehicle_location.z
        + 5,  # Adjust this value to change the height of the camera
    )
    camera_rotation = carla.Rotation(pitch=-30, yaw=vehicle_rotation.yaw)

    # Set the spectator camera transform
    spectator.set_transform(carla.Transform(camera_location, camera_rotation))


def latlon_to_xy(lat_center, lon_center, lat_point, lon_point):
    # Define a Transverse Mercator projection with the center point as the origin
    projection = pyproj.Proj(
        proj="tmerc", lat_0=lat_center, lon_0=lon_center, ellps="WGS84"
    )

    # Convert the point lat/lon to x, y coordinates relative to the center point
    x, y = projection(lon_point, lat_point)

    return x, y


def xy_to_latlon(lat_center, lon_center, x, y):
    # Define the same Transverse Mercator projection with the center point as the origin
    projection = pyproj.Proj(
        proj="tmerc", lat_0=lat_center, lon_0=lon_center, ellps="WGS84"
    )

    # Convert x, y coordinates back to lat/lon using the inverse of the projection
    lon_point, lat_point = projection(x, y, inverse=True)

    return lat_point, lon_point


def utm_to_carla(utm_x, utm_y):
    lat, lon = utm.to_latlon(utm_x, utm_y, ZONE_NUMBER, ZONE_LETTER)
    local_x, local_y = latlon_to_xy(GNSS_ORIGIN[0], GNSS_ORIGIN[1], lat, lon)

    return local_x, -local_y


def carla_to_utm(x, y):
    lat, lon = xy_to_latlon(GNSS_ORIGIN[0], GNSS_ORIGIN[1], x, -y)
    utm_x, utm_y, _, _ = utm.from_latlon(lat, lon)

    return utm_x, utm_y

def sumo_to_carla(sumo_location, sumo_rotation, shape, offset):
    # SUMO location is [x,y,z], repsernting the head center of the agent
    # SUMO rotation is [slope, angle, 0.0]
    # Shape is [length, width, height]
    # Offset is [x, y, z]
    # Convert SUMO location to Carla location
    sumo_yaw = -1 * sumo_rotation[1] + 90
    carla_location = carla.Location(
        x=sumo_location[0] - math.cos(math.radians(sumo_yaw)) * shape[0] / 2.0 + offset[0],
        y=-(sumo_location[1] - math.sin(math.radians(sumo_yaw)) * shape[0] / 2.0) + offset[1],
        z=sumo_location[2] + offset[2],
    )
    carla_rotation = carla.Rotation(
        pitch=sumo_rotation[0],
        yaw=sumo_rotation[1]-90,
        roll=sumo_rotation[2],
    )
    carla_transform = carla.Transform(carla_location, carla_rotation)
    return carla_transform

def carla_to_sumo(carla_location, carla_rotation, shape, offset):
    # Carla location is [x,y,z], repsernting the head center of the agent
    # Carla rotation is [pitch, yaw, roll]
    # Shape is [length, width, height]
    # Offset is [x, y, z]
    # Convert Carla location to SUMO location
    sumo_yaw = -1 * carla_rotation.yaw
    sumo_location = [
        carla_location.x + math.cos(math.radians(sumo_yaw)) * shape[0] / 2.0 - offset[0],
        -carla_location.y + math.sin(math.radians(sumo_yaw)) * shape[0] / 2.0 - offset[1],
        carla_location.z - offset[2],
    ]
    sumo_rotation = [
        carla_rotation.pitch,
        carla_rotation.yaw+90,
        carla_rotation.roll,
    ]
    return sumo_location, sumo_rotation