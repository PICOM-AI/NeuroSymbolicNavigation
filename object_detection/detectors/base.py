# Pure-Python detector API. No ROS imports here.

from dataclasses import dataclass
from typing import List, Optional


@dataclass
class Box2D:
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def cx(self) -> float:
        return 0.5 * (self.x1 + self.x2)

    @property
    def cy(self) -> float:
        return 0.5 * (self.y1 + self.y2)

    @property
    def w(self) -> float:
        return max(0.0, self.x2 - self.x1)

    @property
    def h(self) -> float:
        return max(0.0, self.y2 - self.y1)


@dataclass
class Det2D:
    cls: str
    score: float
    box: Box2D
    # Optional extra fields for later extensibility
    track_id: Optional[int] = None


class DetectorBase:
    """Return a list of Det2D for a BGR image."""
    def setup(self, **kwargs) -> None:
        raise NotImplementedError

    def detect(self, bgr_image) -> List[Det2D]:
        raise NotImplementedError
