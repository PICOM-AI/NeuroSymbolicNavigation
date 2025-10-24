import os
import random
import sys
import pickle
import argparse

from game import Game
from learning import Learning
from instance import create_random_instance, read_from_file

parser = argparse.ArgumentParser(description="Train RL agent on instance")
parser.add_argument("--file", required=True, help="Path to ASP file (e.g., map.lp)")
args = parser.parse_args()
instance = read_from_file(args.file)
print("done reading instance")
learning = Learning()
print("done initializing learning")
learning.learn(instance)
print("done learning")
