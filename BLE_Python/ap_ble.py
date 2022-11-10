"""
Copyright 2021 Samuel Ochoa

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

"""
Modified by Jonathan Valvano, August 16, 2022 
Interacts with the Application Processor project for MSP432/CC2650
0xFFF2, 16-bit value, read only, HalfWordData, gets from switches, toggles green LED
0xFFF3, 32-bit value, write only, WordData, sets LEDs
0xFFF4, notify Switch 1, CCCD=0

"""

"""
This program implements a BLE controller for the MSP432/CC2650. 
The keys are used to read and write characteristics 
The Switch 1 is updated with a notification, switch values are plotted and saved to file. 
"""

"""
Requirements: 
Python 3.9
bleak 0.13.0
pygame 2.0.2

Run the following command to install the libraries
'pip install bleak==0.13.0 pygame==2.0.2'
"""

"""
Usage:
Run the script using the following command
'py ap_ble.py'

Once running it will attempt to connect to the MSP432/CC2650 
and once connected a GUI window will appear plotting Switch1 vs time
Type
 0-7 to set LED color
 s to read switch value
 Esc to quit
Switch1 versus time will also be logged into switch.txt file
"""
#required import statements
#asyncio allows for time slicing
import asyncio
#sys useful functions like sys.exit
import sys
from time import sleep
from bleak import BleakScanner
from bleak import BleakClient
import struct
import pygame

#device name to connect to
# DEVICE_NAME = "Shape the World"
FITNESS_NAME = "Fitness Device"
ROBOT_NAME = "Jacki RSLK 1"
test_button = True
test_fitness = True

#If the device address is known then you can also connect directly via the address instead of using the name
#DEVICE_ADDR = "A0:E6:F8:C4:92:82"

#MSP432/CC2650 characteristic uuids

# Characteristics (Values read by Client)
# HALFWORD_UUID_R = "0000fff2" #read-only characteristic for the HalfWordData data (switches). 
# WORD_UUID_W = "0000fff3" #write-only characteristic for the WordData data (LED). 
TIME_UUID_R = "0000fff2" # read-only characteristic for the WordData data (Time).
SOUND_UUID_R = "0000fff3" # read-only characteristic for the WordData data (Time).
TEMPERATURE_UUID_R = "0000fff4" # read-only characteristic for the HalfWordData data (Temperature).

# Characteristics (Values written by Client)
LED_UUID_W = "0000fff6" # read-only characteristic for the WordData data (LED). 
# DISTANCE_UUID_N = "0000ff7" # notify characteristic Distance that can be subscribed to to get the data.

# Read Notify Characteristics
LIGHT_UUID_N = "0000fffA" # notify characteristic Light that can be subscribed to to get the data.
# JOYSTICK_UUID_N = "0000fffB" # notify characteristic Joystick that can be subscribed to to get the data.
JOYSTICK_X_UUID_N = "0000fffB"
JOYSTICK_Y_UUID_N = "0000fffC" 
SWITCH1_UUID_N = "0000fffD" #notify characteristic Switch1 that can be subscribed to to get the data


# To discover characteristics, 
# connect and then call client.get_charactistics

VENDOR_SPECIFIC_UUID = "-0000-1000-8000-00805f9b34fb" #this is appended to the end of the UUIDs above to create the complete characteristic UUID

#Map keys to binary values. Avoid a lot of if statements. The key pressed indexes into the dictionary to get the value 
# KEYS_MAP = {pygame.K_0:0, pygame.K_1:1, pygame.K_2:2,pygame.K_3:3,pygame.K_4:4,pygame.K_5:5,pygame.K_6:6,pygame.K_7:7, pygame.K_s:11}
KEYS_MAP = {pygame.K_0:0, pygame.K_1:1, pygame.K_2:2,pygame.K_3:3,pygame.K_4:4,pygame.K_5:5,pygame.K_6:6,pygame.K_7:7,pygame.K_s:11,pygame.K_t:12,pygame.K_u:13}

FRAMERATE = 1/30 #set frame rate for screen - 30 fps

#globals to share information between async tasks
client_g = None #store client connection
client_h = None
current_keys_g = 0 #stores the state of the key presses
Dist_g = 0 
Light_g = 0 
Joystick_X_g = 0
Joystick_Y_g = 0
Halfword_g = 0
Word_g = 0
Buffer = []
Time = 0
Semaphore = 0

# Robot
Switch_h = 0


#Helper functions
def notification_handler_dist(sender, data): #Callback function for the subscribed notification. Unpacks the distance data and updates the global distance variable
    global Dist_g
    global Semaphore
    #print("{0}: {1}".format(sender, data))
    RawData  = struct.unpack(">H", data)  # unpack 1 unsigned long
    #print (data)
    #print (RawData)
    Dist_g = RawData[0]  #update global for the main loop
    Semaphore = 1

def notification_handler_light(sender, data): #Callback function for the subscribed notification. Unpacks the light data and updates the global light variable
    global Light_g
    global Semaphore
    RawData  = struct.unpack(">I", data)  # unpack 1 unsigned long
    Light_g = RawData[0]  #update global for the main loop
    Semaphore = 1

def notification_handler_joystick_x(sender, data): 
    global Joystick_X_g
    global Semaphore
    raw  = struct.unpack(">I", data)  # unpack 1 unsigned long
    Joystick_X_g = raw[0]-512  #update global for the main loop
    Semaphore = 1

def notification_handler_joystick_y(sender, data): 
    global Joystick_Y_g
    global Semaphore
    raw  = struct.unpack(">I", data)  # unpack 1 unsigned long
    Joystick_Y_g = raw[0]-512  #update global for the main loop
    Semaphore = 1

def notification_handler_switch(sender, data): #Callback function for the subscribed notification. Unpacks the light data and updates the global light variable
    global Switch_h
    global Semaphore
    RawData  = struct.unpack(">H", data)  # unpack 1 unsigned long
    Switch_h = RawData[0]  #update global for the main loop
    Semaphore = 1

#Connects to device using the address. Can be used in place of connect_name() if the address is already known
async def connect_addr(device_addr):
    client = BleakClient(device_addr)
    await client.connect()
    print("Connected to Shape the World")

    return client

#Connects to device named device_name and returns the client object that represents the connection
async def connect_name(device_name):
    devices = await BleakScanner.discover()

    for d in devices:
        if d.name == device_name:
            print("Found "+d.name)

            # connect to a device
            client = BleakClient(d.address)
            await client.connect()
            print("Connected to "+d.name)

            return client

# Reads the Temperature read only characteristic and updates the global halfword variable. 
async def read_Temperature(client):
    global Halfword_g
    halfword_data = await client.read_gatt_char(TEMPERATURE_UUID_R + VENDOR_SPECIFIC_UUID)
    #print (halfword_data)
    halfword_values = struct.unpack(">H", halfword_data) #unpack one 16-bit unsigned short
    #print (halfword_values)
    Halfword_g = halfword_values[0] #send to global
    print ("Temperature = ", Halfword_g)

# Reads the Time read only characteristic and updates the global word variable. 
async def read_Time(client):
    global Word_g
    word_data = await client.read_gatt_char(TIME_UUID_R + VENDOR_SPECIFIC_UUID)
    #print (halfword_data)
    word_values = struct.unpack(">I", word_data) #unpack one 32-bit unsigned short
    #print (halfword_values)
    Word_g = word_values[0] #send to global
    print ("Time = ", Word_g / 10)  # Time is in 100ms units

#Main fuction that runs in a loop, starts the gui and updates the characteristics
async def pygame_gui():
    global client_g #client conneciton
    global client_h
    global Buffer
    global Time
    global Semaphore
    global current_keys_g #holds the state of the arrow keys to write to the rslk
    current_keys_g_old = current_keys_g #retain the old values so we only transmit if something has changed

    #If client has not been connected then connect via device name
    while client_g == None or client_g.is_connected == False:
        if test_fitness and test_button:
            client_g = await connect_name(FITNESS_NAME) #can be changed to connect_addr(DEVICE_ADDR)
            print(client_g)
            client_h = await connect_name(ROBOT_NAME) #can be changed to connect_addr(DEVICE_ADDR)
            print(client_h)
        elif test_fitness: 
            client_g = await connect_name(FITNESS_NAME) #can be changed to connect_addr(DEVICE_ADDR)
            print(client_g)
        elif test_button: 
            client_g = await connect_name(ROBOT_NAME) #can be changed to connect_addr(DEVICE_ADDR)
            print(client_g)
        else: pass
        #print("Failed to connect, trying again")

    if test_fitness:
        # Subscribe to the notification characteristic. Can be commented out if you want to use the read only distance characteristic
        print("Trying to subscribe to Light data")
        await client_g.start_notify(LIGHT_UUID_N + VENDOR_SPECIFIC_UUID, notification_handler_light)
        print("Subscribed to Light data")

        print("Trying to subscribe to Joystick X data")
        await client_g.start_notify(JOYSTICK_X_UUID_N + VENDOR_SPECIFIC_UUID, notification_handler_joystick_x)
        print("Subscribed to Joystick X data")
        # sleep(0.3)

        print("Trying to subscribe to Joystick Y data")
        await client_g.start_notify(JOYSTICK_Y_UUID_N + VENDOR_SPECIFIC_UUID, notification_handler_joystick_y)
        print("Subscribed to Joystick Y data")

    if test_fitness and test_button:
        print("Trying to subscribe to Switch data")
        await client_h.start_notify(SWITCH1_UUID_N + VENDOR_SPECIFIC_UUID, notification_handler_switch)
        print("Subscribed to Switch data")

    elif test_button:
        print("Trying to subscribe to Switch data")
        await client_g.start_notify(SWITCH1_UUID_N + VENDOR_SPECIFIC_UUID, notification_handler_switch)
        print("Subscribed to Switch data")

    f = open("switch.txt", "a")

    #init screen
    pygame.init()
    size = width, height = 320, 240
    screen = pygame.display.set_mode(size)
    for x in range(width): # 0 to width-1
        Buffer.append(0)    	
    #this array will allow scrolling erase
    Time = 0 
    t = 0

    while True: #Main loop
        #First handle key inputs
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                print("Closing switch.txt file")
                f.close()
                pygame.quit()
                sys.exit()

            elif event.type == pygame.KEYUP:
                current_keys_g = -1
            elif event.type == pygame.KEYDOWN and event.key in KEYS_MAP:
                current_keys_g = KEYS_MAP[event.key] 

        # If the new values are different then we write to the characteristic
        if current_keys_g != current_keys_g_old:

            # Reading
            if current_keys_g == 11: # s
                asyncio.create_task(read_Time(client_g))
            elif current_keys_g == 12: # t
                asyncio.create_task(read_Temperature(client_g))
            # elif current_keys_g == 13: # u
            #     asyncio.create_task(read_Joystick(client_g))
            
            # Writing
            elif ((current_keys_g >= 0) and (current_keys_g < 8)): #0-7
                Word_g = current_keys_g
                print ("LED = ", Word_g)
                value = struct.pack(">h", Word_g) # pack into binary, 1 unsigned long (32bit)
                #print (value)
                asyncio.create_task(client_g.write_gatt_char(LED_UUID_W + VENDOR_SPECIFIC_UUID,value))  # create a task to update LEDs. 

        current_keys_g_old = current_keys_g #retain the old values so we only transmit if something has changed

        # Update the screen on notification
        if Semaphore == 1:
            Semaphore = 0
            Time+=1
            
            # Display Light Value and Draw it 
            # print ("Light =", Light_g)
            '''
            f.write("{}, {}\n".format(Time,Light_g))   	

            y = Light_g / 2;       # Modify this based on the range of values being read
            for i in range(10):			
                t+=1
                if t >= width:
                    t = 0
                pygame.draw.line(screen,pygame.Color(0,0,0),(t,Buffer[t]),(t,Buffer[t]))
                Buffer[t] = y
                pygame.draw.line(screen,pygame.Color(255,255,0),(t,y),(t,y))
            '''
            # Display Joystick Value and Draw it 
            print ("Light =", Light_g)
            print ("Joystick X =", Joystick_X_g, " Joystick Y =", Joystick_Y_g)
            print ("Switch =", Switch_h)
            pygame.display.update() #refresh the screen

        await asyncio.sleep(FRAMERATE) #sleep so the frame rate is set and created tasks can run. this is sleep time is very important


if __name__ == "__main__":
    #client_g = asyncio.run(connect_rslk(DEVICE_NAME))
    asyncio.run(pygame_gui())
