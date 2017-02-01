from pyfbsdk import *

def skew_sym(v):
    '''
    Returns skew-symetric matrix for vector v
    '''
    m = FBMatrix([
        0, -v[2], v[1], 0,
        v[2], 0, -v[0], 0,
        -v[1], v[0], 0, 0,
        0, 0, 0, 1
    ])
    # IMPORTANT!: In MoBu, matrices are row-major, need to transpose it
    m.Transpose()
    return m


def align_matrix(a, b):
    '''
    Returns matrix that rotates vector a onto vector b
    '''
    # Turn them into unit vectors
    a.Normalize()
    b.Normalize()

    v = a.CrossProduct(b)
    s = v.Length()  # Sin of angle
    c = a.DotProduct(b)  # Cos of angle

    # Load identity
    I = FBMatrix()

    # a is prallel to b ( return identity)
    if v.Length() == 0:
        return I

    skew_M = skew_sym(v)

    R = I + skew_M + (skew_M * skew_M) * ((1 - c) / (s * s))
    R[15] = 1  # Can' use 3x3 here
    return R

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

# Modified from rotate function from cheatsheet
def rotate(v1, v2, marker):
    # Define Rotation
    target_M = align_matrix(v1, v2) # Represent rotation as a matrix

    # Take current marker orientation
    cur_Ori = FBMatrix()
    marker.GetMatrix(cur_Ori, FBModelTransformationType.kModelRotation, False)

    # Apply rotation
    final_Ori = target_M * cur_Ori
    ori_vec = FBVector3d()
    FBMatrixToRotation(ori_vec, final_Ori) # Go back to a vector representation
    marker.Rotation = ori_vec
    return

# Return distance between two nodes A & B
def distance(A, B):
    Apos = getPos(A)
    Bpos = getPos(B)
    diff = Apos - Bpos
    return diff.Length()

def getchildcount(node):
    count = 0
    while len(node.Children)!=0:
        node = node.Children[0]
        count += 1
    return count


def getendchild(base):
    node = base
    while len(node.Children)!=0:
        node = node.Children[0]
    return node

def createvector(a, b):
    A = getPos(a)
    B = getPos(b)
    return FBVector3d(A[0]-B[0], A[1]-B[1], A[2]-B[2])

def ccd(goal, base):
    i = 0
    # Move cursor to the end child
    end = getendchild(base)
    cur = end
    while distance(cur, goal) > 0.01 and i < (10*getchildcount(base)):
        # Re-evalute scene
        # If you don't do this result will look weird after a couple of iterations
        FBSystem().Scene.Evaluate()

        # Build vector from pivot to effector
        v1 = createvector(end, cur)
        # Build vector from pivot to target (goal)
        v2 = createvector(goal, cur)
        rotate(v1, v2, cur)
        if cur == base:
            cur = getendchild(base)
        else:
            cur = cur.Parent
        i += 1
    return

def main():
    goal = FBFindModelByLabelName('Goal')
    chain_base = FBFindModelByLabelName('Node')
    
    ccd(goal, chain_base)
    
main()