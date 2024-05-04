class PipelineElement:
    def __init__(self, name, executable, priority=0):
        self.name = name
        self.executable = executable
        self.priority = priority


class Pipeline(list):
    def __init__(self, name, elements):
        self.name = name
        self.extend(elements)
        self.sort(key=lambda e: e.priority)

    def __call__(self, *args, **kwargs):
        success = True
        for element in self:
            output = element.executable(*args, **kwargs)

            # The pipeline will report False if one of the pipeline element returned False during execution.
            if output == False:
                success = False
                return success
        return success

    def hook(self, name, executable, priority=None):
        element = PipelineElement(name, executable, priority=priority)
        self.extend([element])
        self.sort(key=lambda e: e.priority)
