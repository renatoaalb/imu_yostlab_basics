import numpy as np
import utils.serial_operations as serial_op
import utils.quaternion_operations as quaternions_op
import utils.euler_angle_operations as euler_op
import time


# Set parameters for IMU configuration
# streaming commands:
# 0: get tared orientation as quaternions
# 1: get tared orientation as euler angles
# 2: rotation matrix
# 65: get raw gyroscope vector
# 66: get raw accelerometer data

# tipo de dado coletado
type_of_data = 0  

data_strategies = {
    0: {
        "extract": serial_op.extract_quaternions,
        "key": "quaternions",
        "angle_function": quaternions_op.calculate_angle_between_quaternions
    },
    1: {
        "extract": serial_op.extract_euler_angles,
        "key": "euler_vector", # Corrigido para bater com o return da função
        "angle_function": euler_op.calculate_angle_between_euler_angles
    },
    2: {
        "extract": serial_op.extract_rotation_matrix,
        "key": "rotation_matrix", # Corrigido para bater com o return da função
        "angle_function": None
    },
    65: {
        "extract": serial_op.extract_gyro,
        "key": "gyro",
        "angle_function": None
    },
    66: {
        "extract": serial_op.extract_accel,
        "key": "accel",
        "angle_function": None
    }
}

# define como extrair e obter os dados a partir do tipo requerido
current_strategy = data_strategies.get(type_of_data)

# número id das IMUs (estão escritos nas próprios IMUs)
imu_ids = [9, 10]

imu_configuration = {
    "disableCompass": True, #command 109
    "disableGyro": False, # command 107
    "disableAccelerometer": False, #108
    "gyroAutoCalib": True, #command 165
    "filterMode": 1, #command 123
    "tareSensor": True, #command 96
    "tareWithQuaternion": {"imu1": [-0.08303,-0.68177,-0.72074,-0.09250], #command 97
                           "imu2": [-0.03717,-0.69611,-0.71553,-0.04518],
                           #"imu3": [-0.03892,-0.69695,-0.71520,-0.03521], #valores G/T
                           #"imu4": [-0.15181,-0.66783,-0.71024,-0.16283], #valores G/T
                           "imu3": [0.00034,0.32569,0.00566,0.94546],  #deitada
                           "imu4": [0.00539,-0.47953,-0.01110,0.87741], #deitada
                           "imu5": [-0.12033,-0.67836,-0.71320,-0.12918],
                           "imu7": [-0.19449,0.68294,0.68215,-0.17446],
                           "imu8": [-0.11651,-0.68053,-0.71276,-0.12357],
                           "imu9": [-0.19449,0.68294,0.68215,-0.17446],
                           "imu10": [-0.11651,-0.68053,-0.71276,-0.12357]},
    "logical_ids": imu_ids,
    "streaming_commands": [type_of_data, 65, 66, 255, 255, 255, 255, 255], #command 80 - ccepts a list of 8 bytes
    "baudrate": 115200 #command 231
}


# Initialize IMUs
serial_port = serial_op.initialize_imu(imu_configuration)

imu1_values = []
imu2_values = []

timestamps = []

angles = []

# Wait configurations processing to avoid breaking
time.sleep(2)

startTime = time.time()

# No information collected yet
current_imu1 = None
current_imu2 = None

serial_port.reset_input_buffer()
while True:
    try:
        bytes_to_read = serial_port.in_waiting

        # If there are data waiting in dongle, process it
        if bytes_to_read > 0:

            # Obtain data in dongle serial port
            data = serial_port.read(bytes_to_read)
            print(data)
            
            if data[0] != 0 and len(data) <=3:
                print('Corrupted data read.')
            
            #value = current_strategy["extract"](data)
            value = current_strategy["extract"](data) 

            # Check which IMU is sending information
            if data[1] == imu_ids[0]:
                current_imu1 = value.get(current_strategy["key"], [])
                imu1_values.append(current_imu1)
            
            elif data[1] == imu_ids[1]:
                current_imu2 = value.get(current_strategy["key"], [])
                imu2_values.append(current_imu2)
                
            # if possible, calculate the angle between the two IMUs
            if current_strategy["angle_function"] and (current_imu1 is not None) and (current_imu2 is not None):
                timestamp = time.time() - startTime
                timestamps.append(timestamp)
                
                angle = current_strategy["angle_function"](current_imu1, current_imu2)

                angles.append(angle)
                
                print(f"Time: {timestamp:.4f}s | IMU 1: {current_imu1} | IMU 2: {current_imu2} | Angle: {angle:.2f}")

                current_imu1 = None
                current_imu2 = None
    
    except KeyboardInterrupt:            
        print("Finished execution with control + c. ")
        serial_op.stop_streaming(serial_port, imu_configuration['logical_ids'])
        serial_op.manual_flush(serial_port)
        break