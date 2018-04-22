import os
import pathfinder as pf
from utilities.functions import GenerateMotionProfile


SAMPLE_PERIOD = 10                  # MS
DISTANCE = 8.05                     # rotations (825 native units)
MAX_VELOCITY = 4.0                  # rotations / second
MAX_ACCELERATION = 8.0              # rotations / second^2
MAX_JERK = 15.0                     # rotations / second^3
POSITION_UNITS = 1023 * (1 / 10)    # 10-bit ADC / 10-turn pot...rotations
VELOCITY_UNITS = 1023 / 100         # 10-bit ADC / 100ms...natural units of Talon velocity

# Generate the path
info, trajectory = pf.generate([pf.Waypoint(0.0, 0.0, 0.0), pf.Waypoint(DISTANCE, 0.0, 0.0)],
                               pf.FIT_HERMITE_CUBIC, 1000000, SAMPLE_PERIOD / 1000, MAX_VELOCITY,
                               MAX_ACCELERATION, MAX_JERK)

GenerateMotionProfile(os.path.dirname(__file__), "boom_intake_to_scale", trajectory,
                      POSITION_UNITS, VELOCITY_UNITS)
