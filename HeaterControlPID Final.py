from machine import Pin, PWM, Timer, ADC
import time, math

# function displays the temperature or setpoint, up to 2 decimal places, on the 4 digit 7-segment display
def display(temp):
    
    if(temp>9999):
        return
        
    D4 = temp % 10
    D3 = (temp//10) % 10
    D2 = (temp//100) % 10
    D1 = temp//1000
    
    dp = Pin(0, Pin.OUT,Pin.PULL_DOWN)
    d1 = Pin(1, Pin.OUT,Pin.PULL_DOWN)
    d2 = Pin(2, Pin.OUT,Pin.PULL_DOWN) 
    d3 = Pin(3, Pin.OUT,Pin.PULL_DOWN)
    d4 = Pin(4, Pin.OUT,Pin.PULL_DOWN) 
    a = Pin(5, Pin.OUT) 
    b = Pin(6, Pin.OUT) 
    c = Pin(7, Pin.OUT)
    d = Pin(8, Pin.OUT) 
    e = Pin(9, Pin.OUT)
    f = Pin(10, Pin.OUT) 
    g = Pin(11, Pin.OUT) 
    
    pins = [g,f,e,d,c,b,a]
    
    values = {0:[0,1,1,1,1,1,1],
            1:[0,0,0,0,1,1,0],
            2:[1,0,1,1,0,1,1],
            3:[1,0,0,1,1,1,1],
            4:[1,1,0,0,1,1,0],
            5:[1,1,0,1,1,0,1],
            6:[1,1,1,1,1,0,1],
            7:[0,0,0,0,1,1,1],
            8:[1,1,1,1,1,1,1],
            9:[1,1,0,1,1,1,1]
        }
    
    #d3.value(1)
    #d1.value(1)
    #d2.value(1)
    #d4.value(1)
        
    for index, value in enumerate(values[D1]):
        pins[index].value(value)
        
        
    time.sleep(0.002)
        
    for index, value in enumerate(values[D2]):
        pins[index].value(value)
        
    time.sleep(0.002)
        
    for index, value in enumerate(values[D3]):
        pins[index].value(value)
        
    time.sleep(0.002)    
    
    for index, value in enumerate(values[D4]):
        pins[index].value(value)
        d4.value(0)	
        dp.value(0)
    
    time.sleep(0.002)
    
    
# Below initializes variables and constants

adc = ADC(Pin(28))

# Resistance of the fixed voltage in the voltage divider circuit for the thermistor
R_fixed = 20
# Accurate value of the voltage feeding into the thermistor
V_p = 3.3265 

# previous value variable for incremental encoder
prev = 0;
# The pin reading value A of the incremental rotary encoder signal
A = Pin(27, Pin.IN, Pin.PULL_DOWN)
# The pin reading value B of the incremental rotary encoder signal
B = Pin(26, Pin.IN, Pin.PULL_DOWN) 

# Increment/decrement values for the rotary encoder; can change the value to increment/decrement more heavily/lightly
lookup_table = {(0,1): 0.5, (0,2): -0.5,(1,0): -0.5, (1,3): 0.5,(3,1): -0.5, (3,2): 0.5,(2,0): 0.5, (2,3): -0.5}

# Initial setpoint of the desired temperature
setpoint = 40
# boolean value allowing the buffer to determine between Displaying vs. Setting temperature
settingMode = False 

# The pin outputting the signal to the field effect transistor (FET).
fet = PWM(Pin(22))

# frequency of the transistor pin. Note: extra research is necessary to determine optimal frequency.
fet.freq(732) 

# The following two variables stores time values to get changes in time to record data for Temp vs Time
temp_time = 0
startTime = time.time()
prevTime = 0
integralBound = 2147483647 # 32 bit system

PIDPeriod = 1

# PID constants
area = 0
# previous error; used to get changed in error for D
PrevError = 0
# previous time value taken; used to get area time*temp for I
prevPIDTime = 0

lowestRoomT = 18

Kp = 2
Ki = 0.2
Kd = 0

#constant for determining max value of MAX PID value to normalize into duty cycle.
k = 1

# the higher the Kp the closest it is to being accurate, if k approaches 1.
calculatedMaxPID = k*Kp*(setPoint - lowestRoomT)

reachedSetpoint = False

while True:
    
    # Below variables read the thermistor
    V_Rfixed = (adc.read_u16()/65535) * V_p
    R_thermistor = R_fixed* ((V_p/V_Rfixed) - 1)
    
    # Dissipation factor calculation
    current = V_p/(R_fixed + R_thermistor)
    res_heating = 1000*(current**2*R_thermistor)/8
    
    
    # Steinhart-hart calibration for NTC thermistor, 1 measurement/second
    if(time.time()-temp_time > 1):
        
        Temp = (1 / (0.0028427564207573488 + 0.000338027116557912 * math.log(R_thermistor) + 0.00000555342539196426 * (math.log(R_thermistor))**3)) - res_heating - 273.15
        temp_time = time.time()

    tempTime = time.time()
    
    
    # Recording Temperature, Resistance, and Time for tuning and data collection; records ever PIDPeriod sec.
    if((time.time() - prevTime) > PIDPeriod or prevTime == 0):
        
        with open('data.txt', 'w') as file:
            file.write(str(Temp) + ", " + str(tempTime-startTime) + ", " + str(R_thermistor) + "\n")
            
        prevTime = time.time()
        
    
    # Reading and processing from the rotary encoder    
    AB = (A.value() << 1) | B.value()
    
    # if the rotary encoder is rotated ...
    if(AB != prev):  # Note: Do not set the setpoint to a negative value. 
        
        settingMode = True
        
        # increment/decrement the setpoint
        setpoint = setpoint + lookup_table[(prev, AB)]
        
        # display the setpoint value
        display(round(setpoint*100))
        
        prev = AB
        lastSetTime = time.time()
    
    # Displays the setpoint temperature for 2 seconds after the rotary encoder is rotated...
    elif(settingMode):
        
        display(round(setpoint*100))
        
        if(time.time()-lastSetTime > 2): # Can change duration
            settingMode = False
            
    # otherwise, display temperature of thermistor ... (Default)
    else:
        
        display(round(Temp*100))
        
        

    # Every PIDPeriod seconds the PID calculation is taken
    if((time.time() - prevPIDTime) >= PIDPeriod or prevPIDTime == 0):
        
        # Calculating the duty cycle for our heater using PID control. PID tuning
        currentError = setpoint - Temp
        
        if(!reachedSetpoint and currentError >= 0):
            reachedSetpoint = true
            integralBound = area
            
        # use PIDPeriod instead
        # timeChange = (time.time() - prevPIDTime)
        
        area = area + (currentError + prevError) * PIDPeriod/2
        if(area > integralBound):
            area = integralBound
        #d = (currentError - PrevError) / PIDPeriod
        
        prevPIDtime = time.time()
        PrevError = currentError
    
    output = round(Kp*currentError + Ki*area + Kd*d)
    DC = output/calculatedMaxPID
    if(DC > 0.99):
        fet.duty_u16(65535)
    else:
        fet.duty_u16(DC * 65535)



# Below is the code used to calibrate the thermistor.
# with open('tempCalibration3.txt', 'w') as file: #cal1 at 41 C,  #cal2 at 67 C, # cal3 at 21 C
    #for i in range(30):
        # V_Rfixed = (adc.read_u16()/65535) * V_p
        # R_thermistor = R_fixed* ((V_p/V_Rfixed) - 1)
        # print("R: " + str(R_thermistor))
        # file.write(str(R_thermistor) + "\n")
        # time.sleep(1)
# break



