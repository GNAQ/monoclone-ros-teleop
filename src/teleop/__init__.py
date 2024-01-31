import rospy
from . import config
from . import utils


print('[monoclone:__init__] Initializing config...')
config.init_config('peg_ins')
print('[monoclone:__init__] Initialize done.')
