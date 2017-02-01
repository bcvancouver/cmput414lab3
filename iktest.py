from pyfbsdk import *
import sys
sys.path.insert(0, 'your folder path')
import ik

goal = FBFindModelByLabelName('Goal')
chain_base = FBFindModelByLabelName('Node')

ik.ccd(goal, chain_base)