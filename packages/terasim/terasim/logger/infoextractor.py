from abc import ABC


class InfoExtractor(ABC):
    def __init__(self, env):
        """Initialize the inforomation extractor.

        Args:
            env (BaseEnv): The environment.
        """
        self.env = env

    def add_initialization_info(self):
        """Add initialization information.
        """
        pass

    def get_snapshot_info(self, control_info):
        """Get the snapshot information.

        Args:
            control_info (dict): The control information.
        """
        pass

    def get_terminate_info(self, stop, reason, additional_info):
        """Get the terminate information.

        Args:
            stop (bool): True if the simulation stops.
            reason (str): The reason of the termination.
            additional_info (dict): The additional information.
        """
        pass
