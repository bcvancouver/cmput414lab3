from pyfbsdk import *

# Extract position from object
def getPos(node):
    m_pos = FBVector3d()
    node.GetVector(m_pos, FBModelTransformationType.kModelTransformation)
    return m_pos

# Extract rotation matrix from object
def getRotMatrix(name):
    marker = FBFindModelByLabelName(name)
    global_R = FBMatrix()
    local_R = FBMatrix()
    marker.GetMatrix(global_R, FBModelTransformationType.kModelRotation, True)
    marker.GetMatrix(local_R, FBModelTransformationType.kModelRotation, False)
    return

# Rotate target by 90 degrees around Y axis

def rotate(name, direction):
    # Define Rotation
    target_rot = FBVector3d(direction.X, direction.Y, direction.Z)
    target_M = FBMatrix()
    FBRotationToMatrix(target_M, target_rot) # Represent rotation as a matrix

    # Take current marker orientation
    marker = FBFindModelByLabelName(name)
    cur_Ori = FBMatrix()
    marker.GetMatrix(cur_Ori, FBModelTransformationType.kModelRotation, False)

    # Apply rotation
    final_Ori = target_M * cur_Ori
    ori_vec = FBVector3d()
    FBMatrixToRotation(ori_vec, final_Ori) # Go back to a vector representation
    marker.Rotation = ori_vec

# Return distance between two nodes
def distance(A, B):
    Apos = getPos(A)
    Bpos = getPos(B)
    diff = Apos - Bpos
    return diff.Length()

def getchildcount(node):
    print node.Children
    return len(node.Children)

def getendchild(base):
    node = base
    while len(node.Children)!=0:
        node = node.Children[0]
    return node

def ccd(goal, base):
    i = 0
    end = getendchild(base)
    while distance(end, goal) > 0.01 and i < (10*getchildcount(base)):
        # Take current bone
        #print i
        i += 1

def main():
    goal = FBFindModelByLabelName('Goal')
    chain_base = FBFindModelByLabelName('Node')
    
    ccd(goal, chain_base)
    
main()