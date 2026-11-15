"""Run complete scene blocks in an isolated ROS domain."""
import argparse
import json
import os

from run import OUT, fly, runtime


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scenes", type=int, nargs="+")
    args = parser.parse_args()
    formal = OUT/"formal"
    matrix = json.loads((formal/"frozen_matrix.json").read_text())
    binary_hashes = runtime()
    for scene in args.scenes:
        config = matrix[scene-1]
        assert config["scene_id"] == scene
        block = formal/f"scene_{scene:02d}"
        for method in config["order"]:
            folder = block/"runs"/method
            if (folder/"result.json").exists():
                continue
            print("START",block.name,method,"ROS domain",os.environ["ROS_DOMAIN_ID"],flush=True)
            fly(block,method,OUT/"launch",binary_hashes)


if __name__ == "__main__":
    main()
