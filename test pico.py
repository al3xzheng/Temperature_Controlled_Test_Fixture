import serial
import time

# Open the correct COM port (replace 'COM5' with the actual port your Pico is connected to)
ser = serial.Serial('COM5', 9600, timeout=1)

while True:
    # Read data from the Pico
    data = ser.readline().decode('utf-8').strip()  # Read one line of data

    if data:
        print(f"Received from Pico: {data}")  # Print the data received from Pico
    else:
        print("No data received")  # In case no data is available

    # Optionally, send data back to the Pico
    ser.write("Hello from Host!\n".encode('utf-8'))
    
    #time.sleep(2)  # Wait for 2 seconds before checking again