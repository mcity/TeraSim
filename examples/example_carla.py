from pathlib import Path
from terasim.simulator import Simulator
from terasim.physics import CarlaPhysics

current_path = Path(__file__).parent
maps_path = current_path / "maps" / "CarlaTown04"

sim = Simulator(
    sumo_net_file_path=maps_path / "Town04.net.xml",
    sumo_config_file_path=maps_path / "Town04.sumocfg",
    num_tries=10,
    gui_flag=True,
    output_path=current_path / "output" / "0",
    sumo_output_file_types=["fcd_all"],
)
sim.add_plugin(CarlaPhysics(host="192.168.68.105", map="Town04", sync_on_demand=False))
sim.run()
