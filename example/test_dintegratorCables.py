import sys

sys.path.append("./")
import robot_python
import numpy as np
import yaml
from pathlib import Path

np.set_printoptions(linewidth=np.inf)
np.set_printoptions(suppress=True)

sys.path.append("/home/khaledwahba94/imrc/db-CBS/scripts/")
print(sys.path)
# sys.path.append(str(Path(__file__).resolve().parent.parent))

from visualize_dintCables import dintegratorCables_meschatViewer

# state1 = [0.112782,0,0.12406,-0.12406,0,0,0,0]
# state1 = [0.684052,-1.16514e-05,0.162184,0.461334,0.0884134,0.370166,0.101048,0.0242341]
# state1 = [0.791045,-3.44113e-09,-0.156666,3.09972,0.33758,1.73199,-4.1146,2.57581]
state1 = [
    2.836354e-01,
    -2.141536e-01,
    7.140283e-01,
    4.088191e-01,
    3.391446e-01,
    -2.795776e-01,
    -1.372558e00,
    2.405698e00,
]
action1 = [0, 0, 0, 0]

collision_check = True
# Define the environment and robot parameters in a YAML structure
model_yaml = "../../../dynoplan/dynobench/models/DintegratorCables.yaml"

# Save this YAML to a temporary file or use it directly if modifying the function
path_to_env = "../../../example/cables_integrator2_2d_window.yaml"


r = robot_python.robot_factory_with_env(model_yaml, path_to_env)

steps = 101
act_steps = steps - 1
print(int(act_steps / 2))
new_states = np.zeros((steps, 8))
actions = np.zeros((int(act_steps), 4))
# new_states[0] = [-0.0610608,0.0149242,-2.8276,-0.36564,7.48542e-06,1.78233e-06,2.75572e-06,5.18667e-06]
new_states[0] = [-1.0, 0.0, 1.57, -1.57, 0.0, 0.0, 0.0, 0.0]
# ux1
actions_linspace21 = np.linspace(0, 0.07, num=int(act_steps / 2))
actions_linspace22 = np.linspace(0.07, 0, num=int(act_steps / 2))
# ux2
actions_linspace11 = np.linspace(0, -0.02, num=int(act_steps / 2))
actions_linspace12 = np.linspace(0.0 - 0.02, 0, num=int(act_steps / 2))

# uy1
actions_linspace23 = np.linspace(0, 0.0, num=int(act_steps / 2))
actions_linspace24 = np.linspace(0.0, 0, num=int(act_steps / 2))
# uy2
actions_linspace13 = np.linspace(0, 0.0, num=int(act_steps / 2))
actions_linspace14 = np.linspace(0.0, 0, num=int(act_steps / 2))

actions[0 : int(steps / 2), 0] = actions_linspace11  # ux1
actions[int(steps / 2) : :, 0] = actions_linspace12

actions[0 : int(steps / 2), 2] = actions_linspace21  # ux2
actions[int(steps / 2) : :, 2] = actions_linspace22

actions[0 : int(steps / 2), 1] = actions_linspace13  # uy1
actions[int(steps / 2) : :, 1] = actions_linspace14

actions[0 : int(steps / 2), 3] = actions_linspace23  # uy2
actions[int(steps / 2) : :, 3] = actions_linspace24


r = robot_python.robot_factory_with_env(model_yaml, path_to_env)
for k, state in enumerate(new_states):
    if k < new_states.shape[0] - 1:
        next_state = np.zeros(
            8,
        )
        r.step(next_state, state, actions[k], 0.1)
        new_states[k + 1, :] = next_state
# actions += np.random.normal(0,0.1, actions.shape)
# new_states += np.random.normal(0,0.1, new_states.shape)

# Create a dummy result YAML file with state1
if collision_check:
    c = robot_python.CollisionOut()
    r.collision_distance(state1, c)
    path_to_result = "../../../collision_check.yaml"
else:
    path_to_result = "../../../init_guess_cables.yaml"

result_yaml = dict()
result_yaml["result"] = dict()
if collision_check:
    result_yaml["result"]["states"] = [state1]
    result_yaml["result"]["actions"] = [action1]
else:
    result_yaml["result"]["states"] = new_states.tolist()
    result_yaml["result"]["actions"] = actions.tolist()

with open(path_to_result, "w") as file:
    yaml.safe_dump(result_yaml, file, default_flow_style=None)
# Assuming dintegratorCables_meschatViewer is set up to read these files
# You would call the viewer like this:
sys.argv = [
    "dintegratorCables_viewer",  # sys.argv[0], the script name
    "--env",
    path_to_env,
    "--result",
    path_to_result,
    "--output",
    "/home/khaledwahba94/imrc/db-CBS/test_states.html",
]


if collision_check:
    dintegratorCables_meschatViewer([c.p1, c.p2])
else:
    dintegratorCables_meschatViewer()
