from waymo_open_dataset.protos import map_pb2, scenario_pb2
import xml.etree.ElementTree as ET
import warnings
import utm


# Fixme: manually assign id list. Unfinished yet.
ROAD_EDGE_ID_LIST = ['47793', '56740', '48461', '48512', '56570', '44944', '50121', '50153', '50154',
                     '57348', '50093', '47641', '47572', '50080', '56926', '47943',
                     '50122', '44945', '44944']
SPECIAL_ROAD_EDGE_ID_LIST = {'57349': [0,19], '56652': [25,43]}
ROAD_LINE_ID_LIST = ['47794', '47868', '47642', '50094', '48462']
ROAD_LINE_DOUBLE_YELLOW_ID_LIST = ['47642', '50094', '48462']
ROAD_LINE_SINGLE_YELLOW_ID_LIST = ['47794', '47868']
RELATION_TRIM_ID_LIST = {'56650': [0,21], '56569': [0,25], '56739': [0,26], '57136': [18,42], '57422': [18,35], '57347': [18,46], '57047': [18,41]}
RELATION_ADD_ID_LIST = {'56650': [20,39], '57047': [0,19]}

class OSM2Waymo:
    def __init__(self, osm_path):
        self.osm_path = osm_path
        self.map_info = {'node': {}, 'way': {}, 'relation': {}}
        self.read_osm_file()
        self.center_lane = {}
        self.road_line = {}
        self.road_edge = {}

    def read_osm_file(self):
        # find relation id which has a member of type 'way'
        # then find the 'way' id, which gives a series of 'nd' id
        # then find the 'node' id, which gives <tag k='local_x' v='-5.816973' />  <tag k='local_y' v='42.180769' />
        tree = ET.parse(self.osm_path)
        root = tree.getroot()

        for node in root.findall('.//node'):
            node_id = node.get('id')
            lat = node.get('lat')
            lon = node.get('lon')

            # Get tag information
            tag_info = {}
            for tag in node.findall('tag'):
                tag_info[tag.get('k')] = tag.get('v')

            # Store node information
            # REMARK: change local_x and local_y using utm, which are not correct in the original file
            local_x_utm, local_y_utm, _, _ = utm.from_latlon(float(lat), float(lon))
            offset_x = 277546.4056676902
            offset_y = 4686667.009516237
            local_x_utm -= offset_x
            local_y_utm -= offset_y
            tag_info['local_x'] = str(local_x_utm)
            tag_info['local_y'] = str(local_y_utm)
            

            self.map_info['node'][node_id] = {
                'lat': lat,
                'lon': lon,
                'tags': tag_info
            }
            
            

        for relation in root.findall('.//relation'):
            relation_id = relation.get('id')
            members = []
            for member in relation.findall('member'):
                member_info = {
                    'type': member.get('type'),
                    'ref': member.get('ref'),
                    'role': member.get('role')
                }
                members.append(member_info)

            # Get tag information of the relation
            tag_info = {}
            for tag in relation.findall('tag'):
                tag_info[tag.get('k')] = tag.get('v')

            self.map_info['relation'][relation_id] = {
                'members': members,
                'tags': tag_info
            }

        for way in root.findall('.//way'):
            way_id = way.get('id')
            nd_refs = [nd.get('ref') for nd in way.findall('nd')]
            self.map_info['way'][way_id] = nd_refs

        # REMARK: change some points local_x and local_y, which are not correct in the original file
        # self.map_info['node']['56948']['tags']['local_y'] = '48.2'
        # self.map_info['node']['56946']['tags']['local_y'] = '47.8'
        # self.map_info['node']['56944']['tags']['local_y'] = '47.6'
        

    def extract_center_lane(self):
        for relation_id, info in self.map_info['relation'].items():
            left_nodes = []
            right_nodes = []
            for member in info['members']:
                if member['type'] == 'way' and member['role'] == 'left':
                    # print("Relation ID:", relation_id)
                    # print("Way ID (left):", member['ref'])
                    left_nodes = self.map_info['way'][member['ref']]
                    # print(left_nodes)
                if member['type'] == 'way' and member['role'] == 'right':
                    # print("Relation ID:", relation_id)
                    # print("Way ID (right):", member['ref'])
                    right_nodes = self.map_info['way'][member['ref']]
                    # print(right_nodes)
            if len(left_nodes) != len(right_nodes):
                warning_message = f"Relation {relation_id} has unbalanced right and left lane nodes"
                warnings.warn(warning_message, Warning)
            if len(left_nodes) != 0:
                if relation_id in RELATION_TRIM_ID_LIST.keys():
                    lane_start, lane_end = RELATION_TRIM_ID_LIST[relation_id]
                else:
                    lane_start, lane_end = 0, len(left_nodes)
                self.center_lane[relation_id] = []
                for left_node, right_node in zip(left_nodes[lane_start:lane_end], right_nodes[lane_start:lane_end]):
                    # tmp_xy = utm.from_latlon(float(self.map_info['node'][left_node]['lat']),
                    #                          float(self.map_info['node'][left_node]['lon']))
                    # left_x = tmp_xy[0]
                    # left_y = tmp_xy[1]
                    # tmp_xy = utm.from_latlon(float(self.map_info['node'][right_node]['lat']),
                    #                          float(self.map_info['node'][right_node]['lon']))
                    # right_x = tmp_xy[0]
                    # right_y = tmp_xy[1]
                    left_x = self.map_info['node'][left_node]['tags']['local_x']
                    left_y = self.map_info['node'][left_node]['tags']['local_y']
                    right_x = self.map_info['node'][right_node]['tags']['local_x']
                    right_y = self.map_info['node'][right_node]['tags']['local_y']
                    center_x = (float(left_x) + float(right_x)) / 2
                    center_y = (float(left_y) + float(right_y)) / 2
                    self.center_lane[relation_id].append([center_x, center_y])
                if relation_id in RELATION_ADD_ID_LIST.keys():
                    self.center_lane['99'+ relation_id] = []
                    lane_start, lane_end = RELATION_ADD_ID_LIST[relation_id]
                    for left_node, right_node in zip(left_nodes[lane_start:lane_end], right_nodes[lane_start:lane_end]):
                        left_x = self.map_info['node'][left_node]['tags']['local_x']
                        left_y = self.map_info['node'][left_node]['tags']['local_y']
                        right_x = self.map_info['node'][right_node]['tags']['local_x']
                        right_y = self.map_info['node'][right_node]['tags']['local_y']
                        center_x = (float(left_x) + float(right_x)) / 2
                        center_y = (float(left_y) + float(right_y)) / 2
                        self.center_lane['99'+ relation_id].append([center_x, center_y])


    def extract_road_line(self):
        for road_line_id in ROAD_LINE_ID_LIST:
            lane_nodes = self.map_info['way'][road_line_id]
            if len(lane_nodes) != 0:
                self.road_line[road_line_id] = []
                for node in lane_nodes:
                    # tmp_xy = utm.from_latlon(float(self.map_info['node'][node]['lat']),
                    #                          float(self.map_info['node'][node]['lon']))
                    # x = tmp_xy[0]
                    # y = tmp_xy[1]
                    x = self.map_info['node'][node]['tags']['local_x']
                    y = self.map_info['node'][node]['tags']['local_y']
                    self.road_line[road_line_id].append([float(x), float(y)])

    def extract_road_edge(self):
        for road_edge_id in ROAD_EDGE_ID_LIST:
            lane_nodes = self.map_info['way'][road_edge_id]
            if len(lane_nodes) != 0:
                self.road_edge[road_edge_id] = []
                for node in lane_nodes:
                    # tmp_xy = utm.from_latlon(float(self.map_info['node'][node]['lat']),
                    #                          float(self.map_info['node'][node]['lon']))
                    # x = tmp_xy[0]
                    # y = tmp_xy[1]
                    x = self.map_info['node'][node]['tags']['local_x']
                    y = self.map_info['node'][node]['tags']['local_y']
                    self.road_edge[road_edge_id].append([float(x), float(y)])
        for road_edge_id, node_list in SPECIAL_ROAD_EDGE_ID_LIST.items():
            lane_nodes = self.map_info['way'][road_edge_id][node_list[0]:node_list[1]]
            self.road_edge[road_edge_id] = []
            for node in lane_nodes:
                # tmp_xy = utm.from_latlon(float(self.map_info['node'][node]['lat']),
                #                          float(self.map_info['node'][node]['lon']))
                # x = tmp_xy[0]
                # y = tmp_xy[1]
                x = self.map_info['node'][node]['tags']['local_x']
                y = self.map_info['node'][node]['tags']['local_y']
                self.road_edge[road_edge_id].append([float(x), float(y)])

def generate_waymo_scenario(osm_map: OSM2Waymo):
    scenario = scenario_pb2.Scenario()
    scenario.scenario_id = 'Modified_MCity_OneIntersection'
    for lane_idx, lane in osm_map.center_lane.items():
        # Define the points of your lane as MapPoints
        lane_points = []
        for point in lane:
            lane_points.append(map_pb2.MapPoint(x=point[0], y=point[1], z=0.0))
        # Create a LaneCenter message
        lane_center = map_pb2.LaneCenter()
        lane_center.speed_limit_mph = 30.0  # Set the speed limit
        lane_center.type = map_pb2.LaneCenter.TYPE_SURFACE_STREET  # Set the lane type
        lane_center.polyline.extend(lane_points)  # Add the lane points

        # Create a MapFeature message
        map_feature = map_pb2.MapFeature()
        map_feature.id = int(lane_idx)  # Set a unique ID for the lane
        map_feature.lane.CopyFrom(lane_center)  # Set the feature_data to LaneCenter
        scenario.map_features.append(map_feature)
    for lane_idx, lane in osm_map.road_line.items():
        # Define the points of your lane as MapPoints
        lane_points = []
        for point in lane:
            lane_points.append(map_pb2.MapPoint(x=point[0], y=point[1], z=0.0))
        # Create a RoadLine message
        road_line = map_pb2.RoadLine()
        if lane_idx in ROAD_LINE_DOUBLE_YELLOW_ID_LIST:
            # Fixme: solid double yellow does not show, so use solid single yellow instead
            road_line.type = map_pb2.RoadLine.TYPE_SOLID_DOUBLE_YELLOW
        elif lane_idx in ROAD_LINE_SINGLE_YELLOW_ID_LIST:
            road_line.type = map_pb2.RoadLine.TYPE_SOLID_SINGLE_YELLOW
        else:
            road_line.type = map_pb2.RoadLine.TYPE_SOLID_SINGLE_WHITE
        road_line.polyline.extend(lane_points)
        map_feature = map_pb2.MapFeature()
        map_feature.id = int(lane_idx)
        map_feature.road_line.CopyFrom(road_line)
        scenario.map_features.append(map_feature)

    for lane_idx, lane in osm_map.road_edge.items():
        # Define the points of your lane as MapPoints
        lane_points = []
        for point in lane:
            lane_points.append(map_pb2.MapPoint(x=point[0], y=point[1], z=0.0))
        # Create a RoadLine message
        road_edge = map_pb2.RoadEdge()
        road_edge.type = map_pb2.RoadEdge.TYPE_ROAD_EDGE_BOUNDARY
        road_edge.polyline.extend(lane_points)
        map_feature = map_pb2.MapFeature()
        map_feature.id = int(lane_idx)
        map_feature.road_edge.CopyFrom(road_edge)
        scenario.map_features.append(map_feature)
    return scenario


if __name__ == '__main__':
    osm_file_path = 'src/TeraSim_func/maps/lanelet2_mcity_v6-one-intersection_modified.osm'
    mcity_osm_to_map = OSM2Waymo(osm_file_path)
    mcity_osm_to_map.extract_center_lane()
    mcity_osm_to_map.extract_road_line()
    mcity_osm_to_map.extract_road_edge()

    scenario = generate_waymo_scenario(mcity_osm_to_map)
    # Serialize the scenario message
    serialized_scenario = scenario.SerializeToString()
    # Optionally, write the serialized data to a file or send it over the network
    with open("test_scenario", "wb") as f:
        f.write(serialized_scenario)
    # print("Scenario serialized successfully:")
    # print(serialized_scenario)
