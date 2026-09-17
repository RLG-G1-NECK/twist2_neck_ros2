import matplotlib.pyplot as plt
from typing import Literal



class Smoothener:
    """class for smoothening curves using 2nd derivative clamping (other methods can be added later if d2 clamping proves unsatisfactory)"""
    # default d2 values, go tune these yourself. the lower, the smoother + more inertia it has.
    max_pitch_d2 = 0.005
    max_yaw_d2 = 0.01

    def __init__(
            self, 
            prev: tuple[float,float] = (0,0), 
            max_d2_per_step: float | Literal["max_pitch_d2","max_yaw_d2"] 
                = max_pitch_d2):
        self.prev = prev
        match max_d2_per_step:
            case "max_pitch_d2": self.max_d2_per_step = self.max_pitch_d2
            case "max_yaw_d2": self.max_d2_per_step = self.max_yaw_d2
            case _: self.max_d2_per_step = max_d2_per_step

    def smooth_next(self, next_val: float) -> float:
        """clamps the next val such that the value sequence is 'smooth'"""
        d1 = (self.prev[1]-self.prev[0], next_val-self.prev[1])
        d2 = d1[1] - d1[0]
        if abs(d2) > self.max_d2_per_step:
            sign = -1 if d2 < 0 else 1
            self.prev = (
                self.prev[1], self.prev[1] + d1[0] + sign*self.max_d2_per_step)
        else:
            self.prev = (
                self.prev[1], self.prev[1] + d1[1])
        return self.prev[1]

    def smooth_curve(self, vals: list[float]) -> list[float]:
        """smoothens a series of values. use this to debug."""
        smoothed_vals = []
        for val in vals:
            smoothed_vals.append(self.smooth_next(val))
        return smoothed_vals
        

if __name__ == "__main__":
    from twist2_neck_ros2.motion_smoothener.test_log import data

    pitch, _ = map(list, zip(*data))
    
    smoothed_pitch = Smoothener(max_d2_per_step="max_pitch_d2").smooth_curve(pitch)
    pitch_d1 = [t1-t0 for t1,t0 in zip(smoothed_pitch[1:], smoothed_pitch[:-1])]
    pitch_d2 = [dt1-dt0 for dt1,dt0 in zip(pitch_d1[1:],pitch_d1[:-1])]
    plt.plot(pitch, label="raw")
    plt.plot(smoothed_pitch, label="smoothed")
    plt.plot(pitch_d1, label="d1")
    plt.plot(pitch_d2, label="d2")
    plt.legend()
    plt.show()


        
