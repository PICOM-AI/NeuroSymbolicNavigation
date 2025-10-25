
import os
import random
import sys
import pickle

from game import Game
from instance import read_from_file

def default(str):
    return str + ' [Default: %default]'


def readCommand(argv):
    """
    Processes the command used to run pacman from the command line.
    """
    from optparse import OptionParser
    usageStr = """
    USAGE:      python gardener.py <options>
    """
    parser = OptionParser(usageStr)

    parser.add_option('--horizon', dest='horizon', type='int',
                      help=default(
                          'horizon of ASP FR'),
                      metavar='HORIZON', default=5)
    parser.add_option('--radius', dest='radius', type='int',
                      help=default(
                          'radius of ASP FR'),
                      metavar='RADIUS', default=6)
    parser.add_option('--cache', dest='cache', type='int',
                      help=default(
                          'cache of ASP FR'),
                      metavar='RADIUS', default=1)
    parser.add_option('-i', '--interface', dest='interface', action='store_true',
                      help='Show the interface while evaluation',
                      default=False)
    parser.add_option('-r', '--activate-robot', dest='activate_robot', action='store_true',
                      help='Activate the robot during evaluation',
                      default=False)
    options, files = parser.parse_args(argv)
    if len(files) != 1:
        raise Exception('Command line input not understood: ' + str(files))

    args = dict()
    args['interface'] = options.interface
    args['horizon'] = options.horizon
    args['radius'] = options.radius
    args['cache'] = options.cache
    args['activate_robot'] = options.activate_robot
    return args, files

def evaluateFR(files, show, horizon, radius, cache, activate_robot=False):
    MS_CLOCK = 1000
    instance = read_from_file(files[0])
    if activate_robot:
        MS_CLOCK = 30000
        instance.activate_robot("/workspace/maze/maze.yaml")
    learning = pickle.load(
        open(instance.name + '.pkl', 'rb'))
    game = Game(instance, learning, show, True, horizon=horizon + 1, radius=radius, cache=cache)
    game.set_ms_clock(MS_CLOCK)
    game.run()
    instance.print_report('asp')

args, files = readCommand(sys.argv[1:])
evaluateFR(files, args['interface'], args['horizon'], args['radius'], args['cache'], args['activate_robot'])
