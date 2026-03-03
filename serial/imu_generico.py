import biblioteca
import time

imus = [9,10]
serial_port = biblioteca.initialize_dongle(imus)
biblioteca.configure_imu(serial_port, [9])
biblioteca.configure_imu(serial_port, [10])

input("Press Enter to start streaming")
biblioteca.start_streaming(serial_port, imu_ids = [9, 10], frequency = 100, timestamp = True)

list_quaternion1 = []
list_quaternion2 = []
timestamps = []

startTime = time.time()
serial_port.reset_input_buffer()

while True: 
    try:


        quaternion1 = biblioteca.extract_data(serial_port, type_of_data = 0, imu_id = 9)
        quaternion2 = biblioteca.extract_data(serial_port, type_of_data = 0, imu_id = 10)


        timestamp = time.time() - startTime

        #print(quaternion1)
        #print(quaternion2)
        if quaternion1 is not None and quaternion2 is not None:
            list_quaternion1.append(quaternion1)
            #list_quaternion2.append(quaternion2)
            timestamps.append(timestamp)

            print(quaternion1)
            print(quaternion2)

    except KeyboardInterrupt:            
        print("Finished execution with control + c. ")
        biblioteca.stop_streaming(serial_port, imus)
        break
