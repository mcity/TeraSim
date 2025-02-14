from abc import ABC


class Measure(ABC):
    def evaluate(self):
        """Evaluate the measure.

        Returns:
            float: The value of the measure.
        """
        pass
