import machine
import utime

# Define the hardware UART pins
uart = machine.UART(1, baudrate=9600, tx=machine.Pin(4), rx=machine.Pin(5))

# Create a function to convert degrees, minutes, and seconds to decimal degrees
def dms_to_decimal(degrees, minutes, seconds):
    degrees = int(degrees)
    minutes = int(minutes)
    seconds = float(seconds)
    decimal = degrees + (minutes / 60) + (seconds / 3600)
    return decimal

# Create a function to read the GPS data
def read_gps_data():
    data = uart.readline()
    if data:
        try:
            # Decode the GPS data
            data_str = data.strip()

            # Check if the data is a valid GPS sentence
            if data_str.startswith('$GPRMC'):
                # Split the data string into individual values
                values = data_str.split(',')

                # Extract the required values
                latitude = values[2][:2] + '°' + values[2][2:4] + "'" + values[2][4:] + '"' + values[3]
                longitude = values[4][:3] + '°' + values[4][3:5] + "'" + values[4][5:] + '"' + values[5]

                # Convert the latitude and longitude to decimal degrees
                latitude_decimal = dms_to_decimal(values[2][:2], values[2][2:4], values[2][4:])
                longitude_decimal = dms_to_decimal(values[4][:3], values[4][3:5], values[4][5:])

                # Return the latitude and longitude in decimal degrees
                return latitude_decimal, longitude_decimal

        except (UnicodeError, IndexError, ValueError) as e:
            print("Error parsing GPS data:", e)

    # Return None if no valid GPS data is available
    return None, None

# Create a loop to read the GPS data and print it to the console
while True:
    latitude, longitude = read_gps_data()
    print(read_gps_data())
    if latitude is not None and longitude is not None:
        print("Latitude:", latitude)
        print("Longitude:", longitude)
    else:
        print("No values")
    # Wait for 100 milliseconds
    utime.sleep(0.1)



# import machine
# import time

# # Define UART pins
# uart_tx_pin = 4  # TX pin of GPS module connected to GPIO 4 (UART0 TX)
# uart_rx_pin = 5  # RX pin of GPS module connected to GPIO 5 (UART0 RX)
# baud_rate = 9600  # Baud rate for UART communication

# # Initialize UART
# uart = machine.UART(0, baudrate=baud_rate, tx=machine.Pin(uart_tx_pin), rx=machine.Pin(uart_rx_pin))

# # Define LED pins
# led_rx = machine.Pin(2, machine.Pin.OUT)  # RX LED
# led_data = machine.Pin(3, machine.Pin.OUT)  # Data received LED

# # Main loop
# while True:
#     print("Program is running...")

#     # Check UART initialization
#     if uart.any():
#         led_rx.on()  # Turn on RX LED
        
#         gps_data = uart.readline().decode().strip()
#         print("Received GPS data:", gps_data)

#         if gps_data.startswith("$GPGGA"):  # Check if the data is GGA (Global Positioning System Fix Data)
#             led_data.on()  # Turn on data received LED
            
#             print("GPS Data:", gps_data)
#             # Parse GPS data here as needed
#         else:
#             print("Received data is not GGA:", gps_data)
#     else:
#         led_rx.off()  # Turn off RX LED
#         led_data.off()  # Turn off data received LED
        
#         print("No data received")
        
#     time.sleep(0.1)  # Adjust sleep time as needed
