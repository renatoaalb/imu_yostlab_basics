import utils.imu_yostlabs_lara as imu_yostlabs_lara
import time

imus = [9]
input("Press Enter to start configuration")
serial_port = imu_yostlabs_lara.initialize_dongle(imus)
streaming_slots = imu_yostlabs_lara.configure_imu(serial_port, imus)

input("Press Enter to start streaming")
imu_yostlabs_lara.start_streaming(serial_port, imu_ids = imus, frequency = 100, timestamp = True)

current_quaternion1 = current_quaternion2 = None

startTime = time.time()
serial_port.reset_input_buffer()
while True: 
    try:
        data = imu_yostlabs_lara.read_data(serial_port)
        print(data)

        if data is not None:
            quaternion1 = imu_yostlabs_lara.extract_data(data, type_of_data = 0, imu_id = 9)
            quaternion2 = imu_yostlabs_lara.extract_data(data, type_of_data = 0, imu_id = 10)

            if quaternion1 is not None:
                current_quaternion1 = quaternion1

            if quaternion2 is not None:
                current_quaternion2 = quaternion2
                
        if (current_quaternion1 is not None) and (current_quaternion2 is not None):
            timestamp = time.time() - startTime
            print("Quaternion 1:", current_quaternion1, "\nQuaternion 2:", current_quaternion2, "\nTimestamp:",
                  timestamp, "\n")
            current_quaternion1 = current_quaternion2 = None     

    except KeyboardInterrupt:            
        print("Finished execution with control + c. ")
        imu_yostlabs_lara.stop_streaming(serial_port, imus)
        break