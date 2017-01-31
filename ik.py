from pyfbsdk import *

# class object storing rotation degrees on X, Y, Z axis
class direction(x,y,z):
    def __init__(self,x,y,z):
        X=x
        Y=y
        Z=z

# Extract position from object
def getPos(name):
    marker = FBFindModelByLabelName(name)
    m_pos = FBVector3d()
    return marker.GetVector(m_pos, FBModelTransformationType.kModelTransformation)

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

def main():
    # modify rotation
    chain_node.Rotation = r
    # Re-evaluate scene
    FBSystem().Scene.Evaluate()
