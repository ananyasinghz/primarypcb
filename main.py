"""""

                                    ::                                                      
                                    ::                                                      
                                    ::                                                      
                                    ::                                                      
                                    ::                                                      
..    ..........    :.      ::      ::     .........  ..    ..........    ...      .        
::    ::            : .:.   ::     .::.       ::      ::    ::       :    :: :.    :        
::    ::   ..:::    :   .:. ::    ::::::      ::      ::    ::       :    ::   ::  :        
::    ::......::    :      :::    ::::::      ::      ::    ::.......:    ::     :::        
                                  ::::::                                                    
                                  :.::.:                                                    
                     .::::          ::          ::::.                                       
                   .::::::::.       ::       .:::::::::                                     
                   ::::::::::::....::::.....:::::::::::                                     
                    .:::::::::::::::::::::::::::::::::.        
                    
                © Copyright of Ignition Payload Department                     

"""""


from machine import Pin, I2C,SPI,PWM
from mpu6050 import init_mpu6050, get_mpu6050_data
from time import sleep, ticks_ms, ticks_add, ticks_diff
import bme280  
import sdcard
import uos
import math
# from gps import uart as uart_gps

#sd card
# Assign chip select (CS) pin (and start it high)
cs = Pin(13, Pin.OUT)
 #intialize buzzer
# buzzer = Pin(27,Pin.OUT)
BuzzerObj=PWM(Pin(27))

def buzzer(buzzerPinObject,frequency,sound_duration,silence_duration):
    # Set duty cycle to a positive value to emit sound from buzzer
    buzzerPinObject.duty_u16(int(65536*0.2))
    # Set frequency
    buzzerPinObject.freq(frequency)
    # wait for sound duration
    sleep(sound_duration)
    # Set duty cycle to zero to stop sound
    buzzerPinObject.duty_u16(int(65536*0))
    # Wait for sound interrumption, if needed 
    sleep(silence_duration)


def songs():
    buzzer(BuzzerObj,523,0.5,0.1) #C (DO)
    buzzer(BuzzerObj,587,0.5,0.1)#D (RE)
    buzzer(BuzzerObj,659,0.5,0.1) #E (MI)
    buzzer(BuzzerObj,698,0.5,0.1) #F (FA)
    buzzer(BuzzerObj,784,0.5,0.1) #G (SOL)
    buzzer(BuzzerObj,880,0.5,0.1) #A (LA)
    buzzer(BuzzerObj,987,0.5,0.1) #B (SI)


def songserr():
    buzzer(BuzzerObj,523,0.5,0.1) #C (DO)
    buzzer(BuzzerObj,523,0.5,0.1) #C (DO)
    buzzer(BuzzerObj,523,0.5,0.1) #C (DO)
    buzzer(BuzzerObj,523,0.5,0.1) #C (DO)
    buzzer(BuzzerObj,523,0.5,0.1) #C (DO)




# # Intialize SPI peripheral (start with 1 MHz)
spi = SPI(1,
    baudrate=1000000,
    polarity=0,
    phase=0,
    bits=8,
    firstbit=SPI.MSB,
    sck=Pin(10),
    mosi=Pin(11),
    miso=Pin(12))
 
# Initialize SD card
sd = sdcard.SDCard(spi,cs)
 
# Mount filesystem
vfs = uos.VfsFat(sd)
uos.mount(vfs, "/sd")

#Component declaration

#pico led setup
led = Pin(25, Pin.OUT)
led.on()

#component pins
mpu = {
  "sda":20,
  "scl":21,
}
bme = {
  "sda":18,
  "scl":19
}
components = {
  "button":22,
  "ledG":2,
  "ledR":3,
  "buzzer":27
}

#Status checklist
status = {
  "bme":1,
  "mpu":1,
  "gps":1,
  "sd":1
}

#led initialization
ledG = Pin(components["ledG"], Pin.OUT)
ledR = Pin(components["ledR"], Pin.OUT)
#buzzer = Pin(components["buzzer"], Pin.OUT)

#mpu initialization
mpu = I2C(0, scl=Pin(mpu['scl']), sda=Pin(mpu['sda']), freq=400000)
init_mpu6050(mpu)

#bme initialization
i2c=I2C(1,sda=Pin(bme['sda']), scl=Pin(bme['scl']), freq=400000)
bme = bme280.BME280(i2c=i2c)  

#button initialization
button = Pin(components['button'], Pin.IN,Pin.PULL_UP)

#csv file initialization
Objectname=open("data.csv","w")
Objectname.write("TEMP,HUM,PRES,ACCX,ACCY,ACCZ,GYRX,GYRY,GYRZ,VIB\n")
Objectname.flush()

#turn off both LEDs
ledG.value(0)
ledR.value(0)   

#to calculate the vibration magnitude from mpu values
def calculate_vibration(accel_x, accel_y, accel_z, gravity_x, gravity_y, gravity_z):
    # Subtract gravitational acceleration from accelerometer readings
    dynamic_accel_x = accel_x - gravity_x
    dynamic_accel_y = accel_y - gravity_y
    dynamic_accel_z = accel_z - gravity_z
    
    # Calculate magnitude of dynamic acceleration vector
    vibration_magnitude = math.sqrt(dynamic_accel_x**2 + dynamic_accel_y**2 + dynamic_accel_z**2)
    return vibration_magnitude

def mpu_filter():
    # Initialize gyro and accelerometer readings to zero
    gyro_x_offset = 0
    gyro_y_offset = 0
    gyro_z_offset = 0
    accel_x_offset = 0
    accel_y_offset = 0
    accel_z_offset = 0
    
    # Apply low-pass filter to accelerometer readings
    alpha = 0.5
    filtered_accel_x = 0  # Initialize filtered values to zero
    filtered_accel_y = 0
    filtered_accel_z = 0
    
    # Apply low-pass filter to gyro readings
    filtered_gyro_x = 0  # Initialize filtered values to zero
    filtered_gyro_y = 0
    filtered_gyro_z = 0
    
    # Calibrate gyro and accelerometer readings
    for _ in range(1000):
        mpu6050 = get_mpu6050_data(mpu)
        gyro_x_raw, gyro_y_raw, gyro_z_raw = mpu6050['gyro']['x'], mpu6050['gyro']['y'], mpu6050['gyro']['z']
        accel_x_raw, accel_y_raw, accel_z_raw = mpu6050['accel']['x'], mpu6050['accel']['y'], mpu6050['accel']['z']
        
        gyro_x_offset += gyro_x_raw
        gyro_y_offset += gyro_y_raw
        gyro_z_offset += gyro_z_raw
        accel_x_offset += accel_x_raw
        accel_y_offset += accel_y_raw
        accel_z_offset += accel_z_raw
    
    gyro_x_offset /= 1000
    gyro_y_offset /= 1000
    gyro_z_offset /= 1000
    accel_x_offset /= 1000
    accel_y_offset /= 1000
    accel_z_offset /= 1000
    
    # Apply low-pass filter to accelerometer readings
    alpha = 0.5
    filtered_accel_x = alpha * accel_x_offset
    filtered_accel_y = alpha * accel_y_offset
    filtered_accel_z = alpha * accel_z_offset
    
    # Apply low-pass filter to gyro readings
    filtered_gyro_x = alpha * gyro_x_offset
    filtered_gyro_y = alpha * gyro_y_offset
    filtered_gyro_z = alpha * gyro_z_offset
    
    return filtered_accel_x, filtered_accel_y, filtered_accel_z, filtered_gyro_x, filtered_gyro_y, filtered_gyro_z


def error_handling():
    # Check if there are any errors
    has_error = any(value == 0 for value in status.values())
    
    if has_error:
        # Error handling: red light and buzzer
        end_time = ticks_add(ticks_ms(), 30 * 1000)  # 1/2 minute from now
        
        while ticks_diff(end_time, ticks_ms()) > 0:
            # Red light on, green light off
            ledR.on()
            ledG.off()
            
            # beep every 5 seconds
            songserr() # C (DO)
            sleep(5)  # Wait for 5 seconds
            
            # Red light off
            ledR.off()
            sleep(5)  # Wait for 5 seconds

    else:
        # No errors: green light
        end_time = ticks_add(ticks_ms(), 30 * 1000)  # 1/2 minute from now
        
        while ticks_diff(end_time, ticks_ms()) > 0:
            # Green light on, red light off
            ledG.on()
            ledR.off()
            
            sleep(5)  # Wait for 5 seconds
            
            # Green light off
            ledG.off()
            sleep(5)  # Wait for 5 seconds
        
        # Enter the main operation
        return True



def collect_data():
    filtered_accel_x, filtered_accel_y, filtered_accel_z, filtered_gyro_x, filtered_gyro_y, filtered_gyro_z = mpu_filter()
    while True:
        # Check if the button is clicked (pressed)
        if button.value() == 0:
            buzzer.value(1)
            print("Stopping...")
            for i in range(5):
                sleep(1)
                ledG.value(i%2)
                ledR.value(i%2)
            sleep(1)
            Objectname.flush()
            boot_down()
            # Button is pressed, stop data collection
            break
        
        print(bme.values)

        #mpu data reading
        mpu6050 = get_mpu6050_data(mpu)
        gyro_x_raw, gyro_y_raw, gyro_z_raw = mpu6050['gyro']['x'], mpu6050['gyro']['y'], mpu6050['gyro']['z']
        accel_x_raw, accel_y_raw, accel_z_raw = mpu6050['accel']['x'], mpu6050['accel']['y'], mpu6050['accel']['z']
        # Apply low-pass filter to accelerometer readings
        alpha = 0.5
        filtered_accel_x = alpha * filtered_accel_x + (1 - alpha) * accel_x_raw
        filtered_accel_y = alpha * filtered_accel_y + (1 - alpha) * accel_y_raw
        filtered_accel_z = alpha * filtered_accel_z + (1 - alpha) * accel_z_raw
        # Apply low-pass filter to gyro readings
        filtered_gyro_x = alpha * filtered_gyro_x + (1 - alpha) * gyro_x_raw
        filtered_gyro_y = alpha * filtered_gyro_y + (1 - alpha) * gyro_y_raw
        filtered_gyro_z = alpha * filtered_gyro_z + (1 - alpha) * gyro_z_raw

        #Environemnt data
        temp = (float(bme.values[0][:-1]) + float(mpu6050['temp']))/2
        hum = float(bme.values[2][:-1])
        pres = float(bme.values[1][:-3])
        # Example gravitational acceleration components (should be measured or obtained from specifications)
        gravity_x = 0.0  # Example gravitational acceleration component along X-axis
        gravity_y = 0.0  # Example gravitational acceleration component along Y-axis
        gravity_z = 9.8  # Example gravitational acceleration component along Z-axis
        
        # Calculate vibration magnitude
        vibration_magnitude = calculate_vibration(filtered_accel_x, filtered_accel_y, filtered_accel_z, gravity_x, gravity_y, gravity_z)

        pattern = f"{temp},{hum},{pres},{filtered_accel_x},{filtered_accel_y},{filtered_accel_z},{filtered_gyro_x},{filtered_gyro_y},{filtered_gyro_z},{vibration_magnitude}\n"
        pattern = f"{filtered_accel_x},{filtered_accel_y},{filtered_accel_z},{filtered_gyro_x},{filtered_gyro_y},{filtered_gyro_z},{vibration_magnitude}\n"
        print("\n",pattern)
        print(f"x:{filtered_gyro_x},y:{filtered_gyro_y},z:{filtered_gyro_z}")
        with open("/sd/test01.txt", "w") as file:
            file.write(f"{pattern}")
        Objectname.write(pattern)
        Objectname.flush()
        print("Temperature: {:.2f} °C".format(mpu6050['temp']))
        print("Acceleration: X: {:.2f}, Y: {:.2f}, Z: {:.2f} g".format(mpu6050['accel']['x'], mpu6050['accel']['y'], mpu6050['accel']['z']))
        print("Gyroscope: X: {:.2f}, Y: {:.2f}, Z: {:.2f} °/s".format(mpu6050['gyro']['x'],mpu6050['gyro']['y'], mpu6050['gyro']['z']))
        
        
def boot_down():
        ledG.value(1)
        sleep(1)
        ledG.value(0)
        ledR.value(1)
        sleep(1)
        ledR.value(0)
        songs()
        sleep(1) 

def check_sensors():
    mpu6050 = get_mpu6050_data(mpu)

    # Check I2C communication
    try:
        devices = i2c.scan()
        if devices:
            print("I2C Communication OK")
        else:
            raise Exception("No I2C Devices Found")
    except Exception as e:
        print("I2C Communication Error:", e)
        return False
    #Check BME280 sensor
    try:
        bme280 =bme.values
        print("BME280 Sensor OK")
        status["bme"] = 1
        ledG.value(1)
        sleep(1)
        songs()
        sleep(1)

    except Exception as e:
        print("BME280 Sensor Error:", e)
        status["bme"] = 0
        ledR.value(1)
        sleep(1)
        songs()
        sleep(1)
        return False

    #Check MPU6050 sensor
    try:
       mpu6050_acceleration = mpu6050["accel"]
       mpu6050_gyro = mpu6050["gyro"]
       print("MPU6050 Sensor OK")
       status["mpu"] = 1
       ledG.value(1)
       sleep(1)
       songs()
       sleep(1)
    except Exception as e:
       print("MPU6050 Sensor Error:", e)
       status["mpu"] = 0
       ledR.value(1)
       sleep(1)
       songs()
       sleep(1)
       return False
    


    # Check GPS module
    # try:
    #     gps_data = uart_gps.readline().decode("utf-8")
    #     if gps_data.startswith("$GPGGA"):
    #        print("GPS Module OK")
    #        status["gps"] = 1
    #        ledG.value(1)
    #        sleep(500)
    #        ledG.value(1)
    #        sleep(500)
    #        ledG.value(1)
    #        sleep(500)
    #        playsong("C5")
    #     else:
    #       raise Exception("Invalid GPS Data")
    # except Exception as e:
    #     print("GPS Module Error:", e)
    #     status["gps"] = 0
    #     ledR.value(1)
    #     sleep(500)
    #     ledR.value(1)
    #     sleep(500)
    #     ledR.value(1)
    #     sleep(500)
    #     playsong(song)
    #     return False
    #check sd card accessbility

    return True

song = ["C5","C5","C5","P","F5","G5","A5","B5","P","C3"]

def main():
    while True:
        if button.value() == 0:
            print("Initializing...")
            # check_sensors()
            # error_handling()
            collect_data()
                # bz.playsong(song)
                # if check_sensors():
                #     ledG.value(1)
                #     ledR.value(0)
                #     buzzer.value(0)
                    
                #     sleep(2) 
                    # collect_data()
            #     else:
            #         ledG.value(0)
            #         ledR.value(1)
            #         buzzer.value(1)
            # else:
            #     ledG.value(1)
            #     ledR.value(1)
            #     buzzer.value(0)
if __name__ == "__main__":
    main()
