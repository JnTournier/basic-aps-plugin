import time
import sys
from datetime import timedelta
from bluepy import btle

def getTemp(device):
    peripheralDevice = device
    randomAddress = 'random'
    print('[i] Connecting to {}'.format(peripheralDevice))
    try:
        p = btle.Peripheral(peripheralDevice, randomAddress)
    except btle.BTLEDisconnectError:
        print('[w] Your BTLE device is not connected')
        print('[i] Quitting the program')
        exit(1)
        # for svc in p.getServices():
        #     print(f"Services : {svc.uuid.get}")
        
    svc = p.getServiceByUUID('e95d6100-251d-470a-a062-fa1922dfa9a8')
    temperature = svc.getCharacteristics("e95d9250-251d-470a-a062-fa1922dfa9a8")[0]
    
    print('[i] End of the connection...')
    t = temperature.read()
    #a = accelerometer.read()
    
    #print("Temperature: {}".format(ord(t)))
    p.disconnect()
    return ord(t)

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: get-temp.py device")
        exit(1)

    device = sys.argv[1]
    getTemp(device)
