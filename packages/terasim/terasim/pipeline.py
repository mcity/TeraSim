class PipelineElement:
    def __init__(self, name, executable, priority=0):
        """Initialize the pipeline element.

        Args:
            name (str): The name of the pipeline element.
            executable (function): The executable function.
            priority (int, optional): The priority of the pipeline element. Defaults to 0.
        """
        self.name = name
        self.executable = executable
        self.priority = priority


class Pipeline(list):
    def __init__(self, name, elements):
        """Initialize the pipeline.

        Args:
            name (str): The name of the pipeline.
            elements (list): The elements of the pipeline.
        """
        self.name = name
        self.extend(elements)
        self.sort(key=lambda e: e.priority)

    def __call__(self, *args, **kwargs):
        """Execute the pipeline.

        Args:
            *args: The arguments of the pipeline.
            **kwargs: The keyword arguments of the pipeline.

        Returns:
            bool: The success of the pipeline execution.
        """
        success = True
        for element in self:
            output = element.executable(*args, **kwargs)

            # The pipeline will report False if one of the pipeline element returned False during execution.
            if output == False:
                success = False
                return success
        return success

    def hook(self, name, executable, priority=None):
        """Hook a new element into the pipeline.

        Args:
            name (str): The name of the pipeline element.
            executable (function): The executable function.
            priority (int, optional): The priority of the pipeline element. Defaults to None.
        """
        element = PipelineElement(name, executable, priority=priority)
        self.extend([element])
        self.sort(key=lambda e: e.priority)
