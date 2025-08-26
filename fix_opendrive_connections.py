#!/usr/bin/env python3
"""Fix OpenDRIVE file connections based on geographical analysis"""

import xml.etree.ElementTree as ET

def fix_opendrive(input_file, output_file):
    """Fix OpenDRIVE connections based on actual road geometry positions"""
    
    # Parse the XML file
    tree = ET.parse(input_file)
    root = tree.getroot()
    
    # Define correct road connections based on geographical analysis
    road_connections = {
        # Normal roads
        '3': {'pred': None, 'succ': ('junction', '2')},
        '9': {'pred': ('road', '11'), 'succ': ('junction', '1')},
        '4': {'pred': None, 'succ': ('junction', '1')},
        '16': {'pred': ('junction', '1'), 'succ': ('road', '17')},
        '12': {'pred': ('junction', '2'), 'succ': ('road', '13')},
        '13': {'pred': ('road', '12'), 'succ': ('junction', '3')},
        '10': {'pred': ('junction', '3'), 'succ': ('road', '11')},
        '11': {'pred': ('road', '10'), 'succ': ('road', '9')},
        '6': {'pred': ('junction', '3'), 'succ': None},
        '17': {'pred': ('road', '16'), 'succ': ('junction', '4')},
        '14': {'pred': ('junction', '4'), 'succ': None},
        '15': {'pred': ('junction', '4'), 'succ': None},
        
        # Connecting roads (remain as junction="-1")
        '0': {'pred': ('road', '9'), 'succ': ('road', '16')},
        '5': {'pred': ('road', '4'), 'succ': ('road', '16')},
        '2': {'pred': ('road', '3'), 'succ': ('road', '12')},
        '8': {'pred': ('road', '3'), 'succ': None},
        '1': {'pred': ('road', '13'), 'succ': ('road', '10')},
        '7': {'pred': ('road', '13'), 'succ': ('road', '6')},
    }
    
    # Fix road predecessor/successor connections
    print("Fixing road connections...")
    for road in root.findall('.//road'):
        road_id = road.get('id')
        if road_id in road_connections:
            link = road.find('.//link')
            if link is None:
                link = ET.SubElement(road, 'link')
            
            # Update predecessor
            pred_elem = link.find('predecessor')
            new_pred = road_connections[road_id]['pred']
            if new_pred:
                if pred_elem is None:
                    pred_elem = ET.SubElement(link, 'predecessor')
                pred_elem.set('elementType', new_pred[0])
                pred_elem.set('elementId', new_pred[1])
                if new_pred[0] == 'road':
                    pred_elem.set('contactPoint', 'end')
            elif pred_elem is not None:
                link.remove(pred_elem)
            
            # Update successor
            succ_elem = link.find('successor')
            new_succ = road_connections[road_id]['succ']
            if new_succ:
                if succ_elem is None:
                    succ_elem = ET.SubElement(link, 'successor')
                succ_elem.set('elementType', new_succ[0])
                succ_elem.set('elementId', new_succ[1])
                if new_succ[0] == 'road':
                    succ_elem.set('contactPoint', 'start')
            elif succ_elem is not None:
                link.remove(succ_elem)
            
            print(f"  Fixed road {road_id}: pred={new_pred}, succ={new_succ}")
    
    # Fix junction connections
    print("\nFixing junction connections...")
    
    # Define correct junction connections
    junction_connections = {
        '1': [  # Merge junction
            {'incoming': '9', 'connecting': '0', 'contact': 'start', 
             'lanes': [('-1', '-1'), ('-2', '-2'), ('-3', '-3')]},
            {'incoming': '4', 'connecting': '5', 'contact': 'start',
             'lanes': [('-1', '-1')]}
        ],
        '2': [  # Diverge junction
            {'incoming': '3', 'connecting': '2', 'contact': 'start',
             'lanes': [('-1', '-1'), ('-2', '-2'), ('-3', '-3')]},
            {'incoming': '3', 'connecting': '8', 'contact': 'start',
             'lanes': [('-4', '-1'), ('-5', '-2')]}
        ],
        '3': [  # Diverge junction
            {'incoming': '13', 'connecting': '1', 'contact': 'start',
             'lanes': [('-1', '-1'), ('-2', '-2'), ('-3', '-3'), ('-4', '-4')]},  # 1-to-1 mapping
            {'incoming': '13', 'connecting': '7', 'contact': 'start',
             'lanes': [('-4', '-1')]}  # Outermost driving lane to exit
        ],
        '4': [  # Diverge junction
            {'incoming': '17', 'connecting': '14', 'contact': 'start',
             'lanes': [('-1', '-1'), ('-2', '-2'), ('-3', '-3')]},
            {'incoming': '17', 'connecting': '15', 'contact': 'start',
             'lanes': [('-4', '-1')]}
        ]
    }
    
    # Update junction connections
    for junction in root.findall('.//junction'):
        junction_id = junction.get('id')
        if junction_id in junction_connections:
            # Remove all existing connections
            for conn in junction.findall('connection'):
                junction.remove(conn)
            
            # Add correct connections
            for idx, conn_data in enumerate(junction_connections[junction_id]):
                conn = ET.SubElement(junction, 'connection')
                conn.set('incomingRoad', conn_data['incoming'])
                conn.set('id', str(idx))
                conn.set('contactPoint', conn_data['contact'])
                conn.set('connectingRoad', conn_data['connecting'])
                
                # Add lane links
                for from_lane, to_lane in conn_data['lanes']:
                    lane_link = ET.SubElement(conn, 'laneLink')
                    lane_link.set('from', from_lane)
                    lane_link.set('to', to_lane)
                
                print(f"  Junction {junction_id}: Added connection {conn_data['incoming']} -> {conn_data['connecting']}")
    
    # Write the fixed file
    ET.indent(tree, space='    ')
    tree.write(output_file, encoding='utf-8', xml_declaration=True)
    print(f"\nFixed OpenDRIVE file saved to: {output_file}")

if __name__ == "__main__":
    fix_opendrive('test_map_merge_split.xodr', 'test_map_merge_split_fixed.xodr')