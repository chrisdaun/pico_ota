from ota import OTAUpdater
from WIFI_CONFIG import SSID, PASSWORD
firmware_url = "https://github.com/chrisdaun/pico_ota/main"
ota_updater = OTAUpdater(SSID, PASSWORD, firmware_url, "main.py")
ota_updater.download_and_install_update_if_available()




import network
import socket
import time
from uselect import select
from machine import Pin

#WiFi Settings. Change these before uploading to the Pi Pico
WIFI_SSID = 'McMouse'
WIFI_PASSWORD = 'bread555'
WIFI_STATIC_IP = '192.168.86.223'
WIFI_ROUTER_IP = '192.168.86.1'
    
#Set up pins
CLOSED_SENSOR_PIN=20
RELAY_PIN=28
DOOR_STATUS_LED_PIN=22

#Pulse length in ms
PULSE_LENGTH=500

#Homekit target and current states
CURRENT_DOOR_STATE_OPEN = 0
CURRENT_DOOR_STATE_CLOSED = 1
CURRENT_DOOR_STATE_OPENING = 2
CURRENT_DOOR_STATE_CLOSING = 3

# Setup pins for relay, sensors and LEDs
wifiLED = Pin('LED', Pin.OUT)
relay = Pin(RELAY_PIN, Pin.OUT)
# The hardware relay module used is active low, setting it high (off)
relay.value(1)
closedSensor=Pin(CLOSED_SENSOR_PIN, Pin.IN, Pin.PULL_UP)
statusIndicator=Pin(DOOR_STATUS_LED_PIN, Pin.OUT)

IGNORE_SENSORS_AFTER_ACTION_FOR_SECONDS=5

#Set initial target and current states
currentState=CURRENT_DOOR_STATE_CLOSED
lastDoorAction=time.time()

# Create WiFi object
wifi = network.WLAN(network.STA_IF)

def connectWifi():
    global wlan
    wifi = network.WLAN(network.STA_IF)
    wifi.active(True)
    wifi.ifconfig((WIFI_STATIC_IP, '255.255.255.0', WIFI_ROUTER_IP, '8.8.8.8'))
    wifi.connect(WIFI_SSID, WIFI_PASSWORD)
    
    max_wait = 10
    while wifi.status() != 3:
        print('waiting for connection. Status: '+str(wifi.status()))
        time.sleep(1)

    print('connected')
    wifiLED.value(1)
    status = wifi.ifconfig()
    ipAddress=status[0]
    print( 'ip = ' + ipAddress )

# Connecting to WiFi
connectWifi()

#Set up socket and start listening on port 80
addr = socket.getaddrinfo(wifi.ifconfig()[0], 80)[0][-1]
s = socket.socket()
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(addr)
s.listen(1)

print('listening on', addr)

def startDoor():
    global currentState
    global lastDoorAction
    
    lastDoorAction=time.time()

    # Activate the relay for the pulse length and de-activate
    # The hardware relay module used is active low, setting it low (on)
    relay.value(0)
    time.sleep_ms(PULSE_LENGTH)
    relay.value(1)
    setCurrentState()    
    return getDoorStatus()

def setCurrentState():
    global currentState
    #Ignore sensors after having started the door for a few seconds to give the door enough time to move away from the sensor
    actionThresholdReached=time.time()>lastDoorAction+IGNORE_SENSORS_AFTER_ACTION_FOR_SECONDS
            
    #If threshold is reached and door is fully closed
    if actionThresholdReached and closedSensor.value()==0:
        currentState=CURRENT_DOOR_STATE_CLOSED
    elif actionThresholdReached and closedSensor.value()==1:
        currentState=CURRENT_DOOR_STATE_OPEN

    if currentState==(CURRENT_DOOR_STATE_CLOSED):
        statusIndicator.value(0)
    else:
        statusIndicator.value(1)

def getDoorStatus():
    global currentState

    #Ensure current state is up to date
    setCurrentState()
    if currentState==CURRENT_DOOR_STATE_CLOSED:
        targetState=CURRENT_DOOR_STATE_OPENING
    else:
        targetState=CURRENT_DOOR_STATE_CLOSING

    return '{"success": true, "currentState": '+str(currentState)+', "targetState": '+str(targetState)+'}'

def returnError(errcode):
    return '{"success": false, "error": "'+errcode+'"}'

#Handle an incoming request
def handleRequest(conn, address):
    print('Got a connection from %s' % str(addr))
    request = conn.recv(1024)
    request = str(request)

    print(request)

    if request.find('/?open')==6:
        response=startDoor()
    elif request.find('/?close')==6:
        response=startDoor()
    elif request.find('/?getstatus')==6:
        response=getDoorStatus()
    else:
        response=returnError('UNKNOWN_COMMAND')

    conn.send('HTTP/1.0 200 OK\r\nContent-type: application/json\r\n\r\n')
    conn.send(response)
    conn.close()

#Main Loop
while True:
    #Check if wifi is connected, if not, reconnect
    if wifi.isconnected() == False:
        print('Connecting wifi...')
        connectWifi()

    #Handle incoming HTTP requests in a non-blocking way
    r, w, err = select((s,), (), (), 1) # type: ignore

    #Is there an incoming request? If so, handle the request
    if r:
        for readable in r:
            conn, addr = s.accept()
            try:
                handleRequest(conn, addr)
            except OSError as e:
                pass