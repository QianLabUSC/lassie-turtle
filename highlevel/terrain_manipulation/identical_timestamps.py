import numpy as np
p = "/home/parnia/Projects/Turtle_workspace/highlevel/terrain_manipulation/data/session_20260117_211251/trial_1.npy"
d = np.load(p, allow_pickle=True).item()
print("timestamps last:", d["timestamps"][-1])
print("robot_time last:", d["robot_state"]["time"][-1])