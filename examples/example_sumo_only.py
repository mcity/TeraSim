from pathlib import Path

from terasim.simulator import Simulator

current_path = Path(__file__).parent
maps_path = current_path / 'maps' / '3LaneHighway'

sim = Simulator(
    sumo_net_file_path = maps_path / '3LaneHighway.net.xml',
    sumo_config_file_path = maps_path / '3LaneHighway.sumocfg',
    num_tries=10,
    gui_flag=False,
    output_path = current_path / "output" / "0",
    sumo_output_file_types=["fcd_all"],
    realtime_flag=False,
)
sim.run()
