from utils.imu_class_yostlabs_lara import IMU, initialize_dongle, start_streaming, stop_streaming
import time

imus = [10]
serial_port = initialize_dongle(imus)
print(serial_port)

imu1 = IMU(serial_port, imus)

input("Press Enter to start configuration")
imu1.configure()

input("Press Enter to start streaming")
start_streaming(serial_port, imu_ids = imus, frequency = 100)

current_button = None

startTime = time.time()
serial_port.reset_input_buffer()
while True: 
    try:
        data = imu1.read_data()
        if data is not None:
            button = imu1.extract_data(data, type_of_data = 250)

            if button is not None:
                current_button = button

        if (current_button is not None):
            timestamp = time.time() - startTime
            print("Button:", current_button, "\nTimestamp:", timestamp, "\n")
            current_button = None   

    except KeyboardInterrupt:            
        print("Finished execution with control + c. ")
        stop_streaming(serial_port, imus)
        break