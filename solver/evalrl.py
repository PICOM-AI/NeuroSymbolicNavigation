
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

    parser.add_option('-i', '--interface', dest='interface', action='store_true',
                      help='Show the interface while evaluation',
                      default=False)

    options, files = parser.parse_args(argv)
    if len(files) != 1:
        raise Exception('Command line input not understood: ' + str(files))

    args = dict()
    args['interface'] = options.interface
    return args, files

def evaluateFR(files, show):
    MS_CLOCK = 1000
    instance = read_from_file(files[0])

    learning = pickle.load(
        open(instance.name + '.pkl', 'rb'))
    game = Game(instance, learning, show, False)
    game.set_ms_clock(MS_CLOCK)
    game.run()
    instance.print_report('asp')

args, files = readCommand(sys.argv[1:])
evaluateFR(files, args['interface'])
