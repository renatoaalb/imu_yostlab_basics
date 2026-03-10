import utils.imu_yostlabs_lara as imu_yostlabs_lara
import time

imus = [10]
serial_port = imu_yostlabs_lara.initialize_dongle(imus)

input("Press Enter to start configuration")
streaming_slots1 = imu_yostlabs_lara.configure_imu(serial_port, imus, show_quaternion=True, show_euler_angle=True, show_button=True)

input("Press Enter to start streaming")
imu_yostlabs_lara.start_streaming(serial_port, imu_ids = imus, frequency = 100)

current_button = None

startTime = time.time()
serial_port.reset_input_buffer()
while True: 
    try:
        data = imu_yostlabs_lara.read_data(serial_port)

        if data is not None:
            button = imu_yostlabs_lara.extract_data(data, type_of_data = 250, imu_id = 10, streamming_slots=streaming_slots1)

            if button is not None:
                current_button = button

        if (current_button is not None):
            timestamp = time.time() - startTime
            print("Button:", current_button, "\nTimestamp:", timestamp, "\n")
            current_button = None   

    except KeyboardInterrupt:            
        print("Finished execution with control + c. ")
        imu_yostlabs_lara.stop_streaming(serial_port, imus)
        break