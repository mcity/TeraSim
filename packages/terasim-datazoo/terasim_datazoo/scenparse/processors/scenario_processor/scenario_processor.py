from .osmic import Osmic
from .waymonic import Waymonic
from .sumonic import Sumonic

class ScenarioProcessor(Osmic, Waymonic, Sumonic):
    def __init__(self,
                 scenario,
                 osmize_config = {},
                 sumonize_config = {},
                 ) -> None:
        print(f"--------------------SCENARIO ID: {scenario.scenario_id}-------------------")
        self.scenario = scenario
        # Osmic.__init__(self, scenario, osmize_config)
        Waymonic.__init__(self,scenario)
        Sumonic.__init__(self, scenario, self.lanecenters, sumonize_config)
