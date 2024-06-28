import sys
sys.path.append('./')
import robot_python
import numpy as np
import yaml
from pathlib import Path

sys.path.append("/home/khaledwahba94/imrc/db-CBS/scripts/")
print(sys.path)
# sys.path.append(str(Path(__file__).resolve().parent.parent))

from visualize_dintCables import dintegratorCables_meschatViewer

# state1 = [0., 0., np.pi, np.pi, 0., 0., 0., 0.]
# state1 = [0.0, 0.0, 1.1, 0.1, 0.0, 0.0, 0.0, 0.0]
# state1=[9.531649e-01,-3.575269e-02,1.066078e+00,-1.173840e+00,1.479285e+00,-5.056528e-03,1.296440e-02,-1.001428e-02]
state1 = [2.0, 0.0, 0.0, 3.14, 0.0, 0.0, 0.0, 0.0]
# state1 = [2.517053e-01,4.858636e-04,-5.677252e-04,3.141148e+00,9.611156e-01,3.549984e-03,-1.887125e-03,-3.511123e-03]
# state1 = [-0.366834,0,-0.403518,0.403518,0,0,0,0]
# state1 = [-0.366836,4.12133e-06,-0.40351,0.403519,4.77919e-06,8.64969e-06,-5.33762e-06,8.53152e-06]
# state1 = [1.172505e-01,-4.482420e-01,2.572899e-01,-3.462938e-01,6.246282e-02,-4.084694e-01,2.260652e+00,-1.251639e+00]
# state1 = [0.366834,0,0.403518,-0.403518,0,0,0,0]
# state1 = [1.172505e-01,-4.482420e-01,2.572899e-01,-3.462938e-01,6.246282e-02,-4.084694e-01,2.260652e+00,-1.251639e+00]
# state1 = [-0.12782,0,-0.140602,0.140602,0,0,0,0]
# state1 = [-0.799499,0,-0.879449,0.879449,0,0,0,0]
# state1 = [-0.784461,0,-0.862907,0.862907,0,0,0,0]
# state1 = [-0.799499,0,-0.879449,0.879449,0,0,0,0]
# state1 = [0,0,0,0.989724,0,0,0,0]
# cablePos1 = [-1.33314, 0.348981, 0]
# cableQuat1 = [0.908359, 0, 0, -0.418192]
# cablePos2 = [-1.40636, 0.189934, 0]
# cableQuat1 = []
# Define the environment and robot parameters in a YAML structure
model_yaml = "/home/khaledwahba94/imrc/db-CBS/dynoplan/dynobench/models/DintegratorCables.yaml"

# Save this YAML to a temporary file or use it directly if modifying the function
path_to_env = "/home/khaledwahba94/imrc/db-CBS/example/cables_integrator2_2d_window.yaml"

# Create a dummy result YAML file with state1
path_to_result = "temp_result.yaml"
result_yaml = """
states: [{}]
""".format(state1)

with open(path_to_result, "w") as file:
    file.write(result_yaml)

# Assuming dintegratorCables_meschatViewer is set up to read these files
# You would call the viewer like this:
sys.argv = [
    "dintegratorCables_viewer",          # sys.argv[0], the script name
    '--env', path_to_env,
    '--result', path_to_result,
    '--output', '/home/khaledwahba94/imrc/db-CBS/test_states.html'
]

r = robot_python.robot_factory_with_env(model_yaml, path_to_env)
c = robot_python.CollisionOut()

# state = r.get_info(state1)
print(state1)
r.collision_distance(state1, c)
# print("dis: ", c.p1 - c.p2)
# print(c.distance, c.p1, c.p2)

dintegratorCables_meschatViewer([c.p1, c.p2])


# def loadyaml(file_in):
#     with open(file_in, "r") as f:
#         file_out = yaml.load(f,Loader=yaml.CSafeLoader)
#     return file_out

# from viewer.DintegratorCables_viewer import Robot, Visualizer

# env_file = "../../../../deps/dynoplan/dynobench/envs/DintegratorCables/test_col.yaml"
# model_file = "../../../../deps/dynoplan/dynobench/models/DintegratorCables.yaml"

# r = robot_python.robot_factory_with_env(model_file, env_file)




# env = loadyaml(env_file)
# start = env["robots"][0]["start"]
# state1 = start.copy()
# state1[0:2] = np.array([0.5,0])
# print('state1: ',state1)
# state1 = [9.531794e-01,-3.570763e-02,1.066121e+00,-1.173748e+00,1.479289e+00,-5.044048e-03,1.295211e-02,-9.990441e-03]

# # state1 = env["robots"][0]["state1"]
# # state2 = env["robots"]["state2"]


# # print(r.get_x_lb())
# # print(r.get_x_ub())
# # print(r.get_nx())
# # print(r.get_nu())

# print(r.get_x_desc())
# print(r.get_u_desc())

print(state1)
c = robot_python.CollisionOut()

# # print(c)
r.collision_distance(state1, c)
print(c.distance, c.p1, c.p2)


# r.collision_distance(state1, c)
# # print(c.distance, c.p1, c.p2)
