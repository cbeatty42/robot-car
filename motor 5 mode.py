import serial
import threading
import queue
import time

class ArduinoCommunication:
    def __init__(self, port='/dev/ttyACM1', baudrate=9600):
        self.ser = serial.Serial(port, baudrate)
        self.read_queue = queue.Queue()
        self.write_queue = queue.Queue()
        
    def read_from_arduino(self):
        while True:
            if self.ser.in_waiting:
                message = self.ser.readline().decode('utf-8').rstrip()
                self.read_queue.put(message)
                
    def write_to_arduino(self):
        while True:
            message = self.write_queue.get()
            self.ser.write(message.encode('utf-8'))

    def start_communication(self):
        read_thread = threading.Thread(target=self.read_from_arduino)
        write_thread = threading.Thread(target=self.write_to_arduino)
        
        read_thread.start()
        write_thread.start()

if __name__ == "__main__":
    arduino = ArduinoCommunication()

    try:
        arduino.start_communication()
        
        while True:
            # Example: read from Arduino
            if not arduino.read_queue.empty():
                received_message = arduino.read_queue.get()
                print(f'Received: {received_message}')
            
            control_value = int(input("Enter an integer 1-5 for motor control"))
            print("Control value", control_value)

            # Example: write to Arduino
            if control_value == 1:#condition 1
                message_to_send = "D1"
                arduino.write_queue.put(message_to_send)
            elif control_value == 2:#condition 2
                message_to_send = "D2"
                arduino.write_queue.put(message_to_send)
            elif control_value == 3:#condition 3
                message_to_send = "D3"
                arduino.write_queue.put(message_to_send)
            elif control_value == 4:#condition 4
                message_to_send = "D4"
                arduino.write_queue.put(message_to_send)
            elif control_value == 5:#condition 5
                message_to_send = "D5"
                arduino.write_queue.put(message_to_send)
            else:
                print("Value must be between 1 and 5")

    except KeyboardInterrupt:
        print("Keyboard interrupt detected. Exiting...")
    finally:
        arduino.ser.close()