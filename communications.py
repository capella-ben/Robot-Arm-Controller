import serial
import serial.tools.list_ports
import time


class COMMUNICATIONS: 

    def __init__(self, baudRate = 115200, debug=False):
        self.debug = debug
        port = self.find_com_port()
        if not port:
            raise Exception("No suitable device found!")
        
        self.ser = serial.Serial(port, baudRate, timeout=1)
        if self.debug: print(f"Connected to {port}")
        time.sleep(2)  # Allow time for the connection to establish


    def send_commandxx(self, command):
        self.ser.write((command + '\n').encode('utf-8'))

        """response = self.ser.readline().decode('utf-8').strip()
        time.sleep(0.1)
        if response:
            if self.debug: print("Response from device:", response)
        """
        # Read any available data for up to 1 second
        timeout = time.time() + 1
        while time.time() < timeout:
            if self.ser.in_waiting:
                response = self.ser.readline().decode('utf-8').strip()
                if response:
                    print("Response:", response)
            time.sleep(0.1)

    def send_command(self, command, timeout=30):
        self.ser.write((command + '\n').encode('utf-8'))

        # Wait for "done" response with timeout
        start_time = time.time()
        while True:
            if time.time() - start_time > timeout:
                raise TimeoutError(f"Command '{command}' timed out waiting for 'done' response")
                
            if self.ser.in_waiting:
                response = self.ser.readline().decode('utf-8').replace("\n", '')
                print(response)
                if response:
                    if response == "!!!":
                        return
            time.sleep(0.03)


    def close_connection(self):
        self.ser.close()    


    def find_com_port(self):
        ports = serial.tools.list_ports.comports()
        
        for port in ports:
            # You can modify these conditions based on your device's characteristics
            # Common ways to identify the device:
            # 1. By USB VID:PID (Vendor ID and Product ID)
            # if port.vid == 0x0483 and port.pid == 0x5740:  # Example VID:PID
            
            # 2. By manufacturer or description
            # if "Arduino" in port.manufacturer:  # Example for Arduino
            
            # 3. By USB device description
            if port.interface == 'FT232R USB UART':  # Generic USB detection
                if self.debug:
                    print(f"Found device: {port.device}")
                    print(f"   Description: {port.description}")
                    print(f"   Manufacturer: {port.manufacturer}")
                    print(f"   Hardware ID: {port.hwid}")
                return port.device

        return None

if __name__ == "__main__":
    sorterComms = COMMUNICATIONS(debug=False)
    try:
        while True:
            command = input("robot> ")
            sorterComms.send_command(command)

    except KeyboardInterrupt:
        print("\nExiting program.")
    finally:
        sorterComms.close_connection()

