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
# 37: get all corrected component sensor data
# 38: get corrected gyro rate
# 39: get corrected accelerometer vector
# 65: get raw gyroscope vector
# 66: get raw accelerometer data

# Máximo número de bites lidos no buffer
MAXIMUM_BYTES = 300 

# número id das IMUs (estão escritos nas próprios IMUs)
imu_ids = [9]

# Bloco para
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
    "streaming_commands": [0, 38, 39, 255, 255, 255, 255, 255], #command 80 - ccepts a list of 8 bytes
    "baudrate": 115200, #command 231,
    "timestamp": True
}


# Initialize IMUs
serial_port = serial_op.initialize_imu(imu_configuration)

quaternions = []
euler_angles = []
gyro = []
accelerometer = []

timestamps = []

angles = []

# Wait configurations processing to avoid breaking
time.sleep(2)

startTime = time.time()


serial_port.reset_input_buffer()
isFirstLoop = True
while True:
    try:
        
        bytes_to_read = serial_port.in_waiting

         # provavelmente não terá mais de 300 bytes para ler, mas se as informações começarem a faltar, aumente o número máximo de bytes
        if bytes_to_read > MAXIMUM_BYTES:
            serial_port.reset_input_buffer()
        
        # If there are data waiting in dongle, process it
        if bytes_to_read > 0:

            # Obtain data in dongle serial port
            data = serial_port.read(bytes_to_read)
            
            if data[0] != 0 and len(data) <=3:
                print('Corrupted data read.')

            # Check which IMU is sending information
            if data[1] == imu_ids[0]:
                #pegar informações
                quaternion_value = serial_op.extract_quaternions(data, 0)
                #euler_angle_value = serial_op.extract_euler_angles(data, 0)
                accelerometer_value = serial_op.extract_accel(data, 1)
                gyro_value = serial_op.extract_gyro(data, 2)
                

                quaternions.append(quaternion_value)
                #euler
                accelerometer.append(accelerometer_value)
                gyro.append(gyro_value)
                # accelerometer.append(accelerometer_value)

                timestamp = time.time() - startTime
                timestamps.append(timestamp)
                print(f"Time: {timestamp:.4f}s | Quaternion: {quaternion_value} | Accelerometer: {accelerometer_value}",
                #       f"|\n Gyro : {gyro_value}") 
                #print(f"Time: {timestamp:.4f}s | Euler angle: {euler_angle_value} | Accelerometer: {accelerometer_value}",
                        f"|\n Gyro : {gyro_value}")

    
    except KeyboardInterrupt:            
        print("Finished execution with control + c. ")
        serial_op.stop_streaming(serial_port, imu_configuration['logical_ids'])
        serial_op.manual_flush(serial_port)
        break
