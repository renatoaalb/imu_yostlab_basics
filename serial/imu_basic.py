import utils.imu_yostlabs_lara as imu_yostlabs_lara
import time

imus = [9]
input("Press Enter to start configuration")
serial_port = imu_yostlabs_lara.initialize_sensor(imus)
streaming_slots1 = imu_yostlabs_lara.configure_imu(serial_port, imus, show_euler_angle=False, show_quaternion=False, 
                                                   show_accel=True, show_gyro=True)

input("Press Enter to start streaming")
imu_yostlabs_lara.start_streaming(serial_port, imu_ids = imus, frequency = 100, timestamp = True)

current_accel = None

startTime = time.time()
serial_port.reset_input_buffer()
while True: 
    try:
        data = imu_yostlabs_lara.read_data(serial_port)
        

        if data is not None:
            print(data)
            accel = imu_yostlabs_lara.extract_data(data, type_of_data = 39, imu_id = imus, streaming_slots=streaming_slots1, usb = True)

            if accel is not None:
                current_accel = accel
                
        if (current_accel is not None):
            timestamp = time.time() - startTime
            #print("Accel:", current_accel, "\nTimestamp:", timestamp, "\n")
            current_accel = None

    except KeyboardInterrupt:            
        print("Finished execution with control + c. ")
        imu_yostlabs_lara.stop_streaming(serial_port, imus)
        break