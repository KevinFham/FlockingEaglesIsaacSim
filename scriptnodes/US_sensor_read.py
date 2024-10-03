'''
 US_sensor_read.py

 - For reading ultrasonic distance sensors

'''
from omni.isaac.range_sensor._range_sensor import acquire_ultrasonic_sensor_interface

def setup(db: og.Database):
    pass

def cleanup(db: og.Database):
    pass

def compute(db: og.Database):
    db.outputs.Sensor0_out = acquire_ultrasonic_sensor_interface().get_depth_data(str(db.inputs.Sensor0[0]), 0) / 1000 if db.inputs.Sensor0 else -1
    db.outputs.Sensor1_out = acquire_ultrasonic_sensor_interface().get_depth_data(str(db.inputs.Sensor1[0]), 0) / 1000 if db.inputs.Sensor1 else -1
    db.outputs.Sensor2_out = acquire_ultrasonic_sensor_interface().get_depth_data(str(db.inputs.Sensor2[0]), 0) / 1000 if db.inputs.Sensor2 else -1

    return True