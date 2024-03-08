from dataclasses import dataclass
from typing import Callable, Dict, Optional, Sequence, Union

import matplotlib.pyplot as plt
import numpy as np

from . import _disbmp


class State(_disbmp._State):
    def __init__(self, s: Union[np.ndarray, Sequence]):
        if not isinstance(s, np.ndarray):
            s = np.array(s)
        super().__init__(*s)

    def to_vector(self):
        return super().to_vector()


class BoundingBox(_disbmp._BoundingBox):
    def __init__(
        self, s_min: Union[np.ndarray, Sequence], s_max: Union[np.ndarray, Sequence]
    ):
        if not isinstance(s_min, np.ndarray):
            s_min = np.array(s_min)
        if not isinstance(s_max, np.ndarray):
            s_max = np.array(s_max)
        super().__init__(State(s_min), State(s_max))


@dataclass
class TrajectoryPiece:
    traj_piece: _disbmp._TrajectoryPiece

    @classmethod
    def from_raw(cls, raw: _disbmp._TrajectoryPiece):
        return cls(raw)

    def interpolate(self, t: float) -> np.ndarray:
        state: State = self.traj_piece.interpolate(t)
        return state.to_vector()

    def visualize(
        self,
        ax: plt.Axes,
        resolution: float,
        kwargs_plot: Optional[Dict] = None,
        kwargs_scatter: Optional[Dict] = None,
    ):
        if kwargs_plot is None:
            kwargs_plot = {}
        if kwargs_scatter is None:
            kwargs_scatter = {}
        duration = self.traj_piece.duration
        times = np.arange(0, duration, resolution).tolist() + [duration]
        for i in range(len(times) - 1):
            s0 = self.interpolate(times[i])
            s1 = self.interpolate(times[i + 1])
            ax.plot([s0[0], s1[0]], [s0[1], s1[1]], **kwargs_plot)
        s_start = self.interpolate(0)
        s_end = self.interpolate(duration)
        ax.scatter(s_start[0], s_start[1], **kwargs_scatter)
        ax.scatter(s_end[0], s_end[1], **kwargs_scatter)

    def __getstate__(self):
        state_dict = {}
        state_dict["duration"] = self.traj_piece.duration
        state_dict["d"] = self.traj_piece.d
        state_dict["s1_vec"] = self.traj_piece.s1_vec
        return state_dict

    def __setstate__(self, state_dict):
        self.traj_piece = _disbmp._TrajectoryPiece(
            state_dict["duration"], state_dict["d"], state_dict["s1_vec"]
        )


@dataclass
class Trajectory:
    traj: _disbmp._Trajectory

    @classmethod
    def from_raw(cls, raw: _disbmp._Trajectory):
        return cls(raw)

    def interpolate(self, t: float) -> np.ndarray:
        state: State = self.traj.interpolate(t)
        return state.to_vector()

    def get_duration(self) -> float:
        return self.traj.get_duration()

    def get_length(self, n_split: int = 20) -> float:
        return self.traj.get_length(n_split)

    def __getstate__(self):
        state_dict = {}
        raw_pieces = self.traj.pieces
        pieces = [TrajectoryPiece.from_raw(raw_piece) for raw_piece in raw_pieces]
        state_dict["pieces"] = pieces
        return state_dict

    def __setstate__(self, state_dict):
        pieces = state_dict["pieces"]
        raw_pieces = [p.traj_piece for p in pieces]
        self.traj = _disbmp._Trajectory(raw_pieces)


class RRT(_disbmp._RRT):
    def __init__(
        self,
        start: State,
        goal: State,
        is_obstacle_free: Callable[[State], bool],
        bounding_box: BoundingBox,
        t_extension: float,
        resolution: float,
    ):
        super().__init__(
            start, goal, is_obstacle_free, bounding_box, t_extension, resolution
        )

    def solve(self, max_iter: int) -> bool:
        return super().solve(max_iter)

    def get_solution(self) -> Trajectory:
        raw = super().get_solution()
        return Trajectory.from_raw(raw)


class FastMarchingTree(_disbmp._FastMarchingTree):
    def __init__(
        self,
        start: State,
        goal: State,
        is_obstacle_free: Callable[[State], bool],
        bounding_box: BoundingBox,
        resolution: float,
        admissible_cost: float,
        N_sample: int,
    ):
        super().__init__(
            start,
            goal,
            is_obstacle_free,
            bounding_box,
            resolution,
            admissible_cost,
            N_sample,
        )

    def solve(self, max_iter: int) -> bool:
        return super().solve(max_iter)

    def get_solution(self) -> Trajectory:
        raw = super().get_solution()
        return Trajectory.from_raw(raw)

    def get_all_motions(self) -> Sequence[TrajectoryPiece]:
        raws = super().get_all_motions()
        return [TrajectoryPiece.from_raw(raw) for raw in raws]


def resample(
    traj: Sequence[TrajectoryPiece], resolution: float
) -> Sequence[TrajectoryPiece]:
    raws = _disbmp.resample([piece.traj_piece for piece in traj], resolution)
    return [TrajectoryPiece.from_raw(raw) for raw in raws]
