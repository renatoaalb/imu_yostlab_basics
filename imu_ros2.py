#!/usr/bin/env python3

import rclpy  # Import for ROS2
from std_msgs.msg import Float32MultiArray
import sys
import traceback
import numpy as np
import serial_operations as serial_op

imu_ids = [9, 10]

# Set parameters for IMU configuration
# streaming commands:
# 0: get tared orientation as quaternions
# 1: get tared orientation as euler angles
# 2: rotation matrix
# 65: get raw gyroscope vector
# 66: get raw accelerometer data
# 255: no data
# 
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
    "logical_ids": [imu_ids[0], imu_ids[1]],
    "streaming_commands": [0, 255, 255, 255, 255, 255, 255, 255], #command 80 - ccepts a list of 8 bytes
    "baudrate": 115200 #command 231
}


# Initialize read values (necessário para o publish do tópico. Floar32MultiArray é um tipo de mensagem)
#value_read = Float32MultiArray()
# extracted_data1 = Float32MultiArray()
msg1 = Float32MultiArray()
msg2 = Float32MultiArray()
msggyro1 = Float32MultiArray()
msggyro2 = Float32MultiArray()
msgaccel1 = Float32MultiArray()
msgaccel2 = Float32MultiArray()

# Initialize imu's quaternions

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu')  # Node name
        self.get_logger().info('Inicializando nó')

        #Publisher
        self.pub_imu_1 = self.create_publisher(Float32MultiArray, 'imu_1_values', 10)
        self.pub_imu_2 = self.create_publisher(Float32MultiArray, 'imu_2_values', 10)

        #Publisher_Gyros
        # self.gyro_1 = self.create_publisher(Float32MultiArray, 'gyro_1', 10)
        # self.gyro_2 = self.create_publisher(Float32MultiArray, 'gyro_2', 10)

        #Publisher_Accel
        # self.accel_1 = self.create_publisher(Float32MultiArray, 'accel_1', 10)
        # self.accel_2 = self.create_publisher(Float32MultiArray, 'accel_2', 10)      

        # Initialize IMUs
        self.serial_port = serial_op.initialize_imu(imu_configuration)

        # Timer to replace rospy.Rate
        frequency = 100
        self.timer = self.create_timer(1.0 / frequency, self.timer_callback)

    def     timer_callback(self):
        try:
            bytes_to_read = self.serial_port.inWaiting
            if bytes_to_read > 0:
                data = self.serial_port.read(bytes_to_read)
                #print(data)
                if data[0] != 0 and len(data) <= 3:
                    self.get_logger().warn(f'Corrupted data read from IMU.')

                if data[1] == imu_ids[0]:
                    extracted_data1 = serial_op.extract_quaternions(data)
                    quaternions1 = extracted_data1.get('quaternions',[])
                    quaternions1 = [float(q) for q in quaternions1 if isinstance(q, (int, float))]
                    msg1.data = quaternions1
                    self.pub_imu_1.publish(msg1)

                    # extracted_gyro1 = serial_op.extract_gyro(data)
                    # gyro1 = extracted_gyro1.get('gyro',[])
                    # gyro1 = [float(q) for q in gyro1 if isinstance(q, (int, float))]
                    # msggyro1.data = gyro1
                    # self.gyro_1.publish(msggyro1)

                    # extracted_accel1 = serial_op.extract_accel(data)
                    # accel1 = extracted_accel1.get('accel',[])
                    # accel1 = [float(q) for q in accel1 if isinstance(q, (int, float))]
                    # msgaccel1.data = accel1
                    # self.accel_1.publish(msgaccel1)

                elif data[1] == imu_ids[1]:
                    extracted_data2 = serial_op.extract_quaternions(data)
                    quaternions2 = extracted_data2.get('quaternions',[])
                    quaternions2 = [float(q) for q in quaternions2 if isinstance(q, (int, float))]
                    msg2.data = quaternions2
                    self.pub_imu_2.publish(msg2)

                    # extracted_gyro2 = serial_op.extract_gyro(data)
                    # gyro2 = extracted_gyro2.get('gyro',[])
                    # gyro2 = [float(q) for q in gyro2 if isinstance(q, (int, float))]
                    # msggyro2.data = gyro2
                    # self.gyro_2.publish(msggyro2)

                    # extracted_accel2 = serial_op.extract_accel(data)
                    # accel2 = extracted_accel2.get('accel',[])
                    # accel2 = [float(q) for q in accel2 if isinstance(q, (int, float))]
                    # msgaccel1.data = accel2
                    # self.accel_2.publish(msgaccel2)

        except KeyboardInterrupt:
            self.get_logger().info("Finished execution with control + C")
            serial_op.stop_streaming(serial_port, imu_configuration['logical_ids'])
            serial_op.manual_flush(serial_port)
            rclpy.shutdown()

        except Exception as error:
            self.get_logger().error('Unhandled exception.')
            self.get_logger().error(str(error))
            self.get_logger().error(traceback.format_exc())
            serial_op.streaming(serial_port, imu_configuration['logical_ids'])

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    try:
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        imu_node.get_logger().info('Node stopped cleanly')
    except Exception as e:
        imu_node.get_logger().error(f'Error: {e}')
    finally:
        imu_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

