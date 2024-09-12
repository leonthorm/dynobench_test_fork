import sys

sys.path.append("./")
import robot_python
import numpy as np
import yaml
from pathlib import Path
import argparse

np.set_printoptions(linewidth=np.inf)
np.set_printoptions(suppress=True)

sys.path.append("/home/khaledwahba94/imrc/db-CBS/scripts/")
print(sys.path)
# sys.path.append(str(Path(__file__).resolve().parent.parent))

from visualize_dintCables import dintegratorCables_meschatViewer



def main():
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

    collision_check = False

    parser = argparse.ArgumentParser()
    parser.add_argument('--env', type=str, help="environment")
    parser.add_argument('--init_guess', type=str, help="dbcbs sol")
    parser.add_argument('--model', type=str, help="DintegratorCables model in dynobench path")
    parser.add_argument('--result', type=str, help="output.yaml")
    parser.add_argument('--output', type=str, help="output.html")
    args = parser.parse_args()
    
    path_to_env = args.env
    path_to_init_guess = args.init_guess
    model_yaml = args.model
    path_to_result = args.result
    path_to_vis = args.output
    #load env

    with open(path_to_env, "r") as f: 
        env = yaml.safe_load(f)
    start = env["robots"][0]["start"]
    with open(path_to_init_guess, "r") as f:
        init_guess = yaml.safe_load(f)
    
    states     = np.array(init_guess["result"]["states"])
    num_states = states.shape[0]
    new_states = np.zeros((states.shape[0],8))
    actions    = np.array(init_guess["result"]["actions"])
    
    new_states[0] = np.array(start)

    r = robot_python.robot_factory_with_env(model_yaml, path_to_env)

    if not collision_check:
        for i in range(num_states-1):
            next_state = np.zeros(8,)
            state = states[i]
            print("state: ", state)
            print("action: ", actions[i])
            r.step(next_state, state, actions[i], 0.1)
            print("next_state", next_state)
            new_states[i + 1, :] = next_state 

    # Create a dummy result YAML file with state1
    if collision_check:
        c = robot_python.CollisionOut()
        r.collision_distance(state1, c)
        # path_to_result = "../../../collision_check.yaml"

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
        path_to_vis,
    ]


    if collision_check:
        dintegratorCables_meschatViewer([c.p1, c.p2])
    else:
        dintegratorCables_meschatViewer()



if __name__ == "__main__":
    main()