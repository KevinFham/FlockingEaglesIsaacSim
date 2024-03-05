def quaternion_to_euler_x(x, y, z, w):
    eul_x = np.arctan2(
        2.0 * (w * x + y * z), 
        1.0 - 2.0 * (x * x + y * y)
    )
    
    return eul_x

def setup(db: og.Database):
    pass

def cleanup(db: og.Database):
    pass

def compute(db: og.Database):

    db.outputs.euler_x = quaternion_to_euler_x(
        db.inputs.quaternion_WXYZ[1],
        db.inputs.quaternion_WXYZ[2],
        db.inputs.quaternion_WXYZ[3],
        db.inputs.quaternion_WXYZ[0],
    )

    return True
