'''
Serial communication with air monitoring equipment.

Mårten Spanne 2018-05-32

Reviderad   2020-08-12 (loggningsfunktionen fungerar)
            2020-11-28 (läser in inloppsport/mätpunkt)
            2020-12-29 (anpassad för mätvagn 4)
            2021-01-05 (multipel loggning och initieringfil implementerade)

Att göra:
* Logga mer sällan, samla data i matris för att sedan spara
    t ex en gång per 10 minuter.
* Använd interrupt för loggningen av data respektive instrumentparametrar
ok Logga instrumentparametrar, t ex 1 gång per 10 eller 15 minuter
ok Läsa felkoder
* Kontrollera BCC och agera om den är fel
* Larma vid felkoder (mail?)
* Subrutin för att släcka felkoder och starta om instrumentet
* Användargränssnitt

'''

import serial, time, datetime, csv, binascii, sys, threading
import RPi.GPIO as GPIO
import configparser, os, os.path

config = configparser.ConfigParser()
config.read('logger_ini.txt')                           # Läs in initieringsfilen
initvars = config['NOx CLD700']
saveFileStub = initvars.get('saveFileStub')             # NOxHTMdata
saveFileMStub = initvars.get('saveFileMStub')           # NOxHTMinstrumentData
day_file_instr_data = initvars.get('day_file_instr_data')# Do/do not change file every day
sample_time = float(initvars.get('sample_time'))        # Sample interval in seconds
sample_time_M = int(initvars.get('sample_time_M'))      # Max 1 hour! Sample interval for instrument data in seconds.
lastdate = datetime.datetime.now().strftime('%d')       # Get current date for savefile
saveFile = saveFileStub + '_' + datetime.datetime.now().strftime('%Y-%m-%d') + '.csv'
saveFileM = saveFileMStub + '_' + datetime.datetime.now().strftime('%Y-%m-%d') + '.csv'
os.chdir('./data')

      
def sendNOxCommand(cmdStr):
    instrID = 1

    # Construct the command sequence
    t = "{:02d}".format(instrID) + cmdStr
    h = []
    h.append(2)                     # STX = \x02
    for i in t:
        h.append(ord(i))
    h.append(3)                     # ETX = \x03

    # Calculate the block control character (BCC)
    bcc = (h[0])
    for n in range(1, len(h)):      # XOR-ing all message characters
        bcc ^= (h[n])
    h.append(bcc)                   # Send complete string!

    n = ser.write(serial.to_bytes(h))
#    print("written {} bytes".format(n))


def getNOxData(cmdStr='RD3'):       # EcoPhysics CLD 700 AL
    global conc, avg, nmeas, fel

    #cmdStr = 'RD3'                  # Report all: NO2, NO, NOx in ppm

    sendNOxCommand(cmdStr)
    time.sleep(0.1)                 # wait some time for the answer to arrive
    i = ser.inWaiting()

    if cmdStr == 'RD3' and i < 25:  # If not all chars have arrived, wait a little
        time.sleep(0.05)
        i = ser.inWaiting()        
    if cmdStr == 'RD3' and i < 25:  # Wait a little longer still...
        time.sleep(0.05)
        i = ser.inWaiting()

    if i > 0:                       # If something, read and handle the input
        reading = ser.read(i)
        if reading[0] == 21:                    # \x15 = 21 = NAK
            print('NAK-Resend!', reading[0:2])  #, end='', flush=True)
            fel += 1
            return -999, -999, -999
        elif reading[0] == 6:                   # 6 = ACK
            # To do: check error codes
            end = reading.find(3)               # Find end of data, ETX
            conc = reading[3:end].decode()
            res = [float(x) for x in conc.replace(',','').split()]  # Convert to list of floats
            # Unless you just want to pass the string on... (to do!)
            for i in range(len(res)):           # Convert to ppb
                res[i] *= 1000
                res[i] = int(res[i])
            nmeas += 1
            return res
        else:
            print(reading)
            fel += 1
            return -999, -999, -999
    else:
        print(clear, 'Nothing received')
        fel += 1
        return -999, -999, -999

def getNOxinstrumentData():
    cmdStr = 'RS'                   # Report instrument status (3x2 bytes: ds (bin), ee (asc), ww (asc))
    strLen = 13
    sendNOxCommand(cmdStr)
    instr_stat = readNOxData(strLen)
    cmdStr = 'RP0'                  # Report sample pressure (4 byte int in mbar)
    strLen = 9
    sendNOxCommand(cmdStr)
    sample_press = readNOxData(strLen)
    cmdStr = 'RP1'                  # Report reactor pressure (4 byte int in mbar)
    strLen = 9
    sendNOxCommand(cmdStr)
    react_press = readNOxData(strLen)
    cmdStr = 'RT0'                  # Report instrument amb temperature (3 byte int in degC)
    strLen = 8
    sendNOxCommand(cmdStr)
    instr_temp = readNOxData(strLen)
    cmdStr = 'RT1'                  # Report PMT temperature (3 byte int in degC)
    strLen = 8
    sendNOxCommand(cmdStr)
    pmt_temp = readNOxData(strLen)
    cmdStr = 'RT2'                  # Report reactor temperature (3 byte int in degC)
    strLen = 8
    sendNOxCommand(cmdStr)
    react_temp = readNOxData(strLen)
    cmdStr = 'RT3'                  # Report converter temperature (3 byte int in degC)
    strLen = 8
    sendNOxCommand(cmdStr)
    conv_temp = readNOxData(strLen)
    cmdStr = 'RF'                   # Report flow (3 byte float in lpm)
    strLen = 8
    sendNOxCommand(cmdStr)
    instr_flow = readNOxData(strLen)
    cmdStr = 'RR'                   # Report measurement range (2 bytes: n (code), m (A/F))
    strLen = 8
    sendNOxCommand(cmdStr)
    instr_range = readNOxData(strLen)

    try:
        # To do: check error codes (p 70)
        if instr_stat[1] != 0x40:                               # Error code (0x40 = @ = no errors)
            print('Error code: {:08b}'.format(instr_stat[1]))   # Print in binary
        
        ## Start to form the datastring
        # Instrument status
        datastr = '{:08b}'.format(instr_stat[3]) + ', '         # instrument status byte d
        datastr += '{:08b}'.format(instr_stat[4]) + ', '        # instrument status byte s
        datastr += instr_stat[6:8].decode() + ', '              # instrument error code
        datastr += instr_stat[9:11].decode() + ', '             # instrument warning code

        # Pressures
        datastr += str(int(sample_press[3:7].decode())) + ', ' 
        datastr += str(int(react_press[3:7].decode())) + ', ' 

        # Temperatures
        datastr += str(int(instr_temp[3:6].decode())) + ', '    # instr temp
        datastr += str(int(pmt_temp[3:6].decode())) + ', '      # PMT temp
        datastr += str(int(react_temp[3:6].decode())) + ', '    # reactor temp
        datastr += str(int(conv_temp[3:6].decode())) + ', '     # converter temp

        # Other
        datastr += str(float(instr_flow[3:6].decode())) + ', '  # instrument flow
        datastr += '{:c}'.format(instr_range[3])                # instrument range
        
        return datastr
    
    except Exception as e:
        print('Error:', str(e))
        return []    

def readNOxData(strLen):            # Should be preceeded by a sendNOxCommand. Returns a list of bytes
    time.sleep(0.1)                 # Wait some time for the answer to arrive
    i = ser.inWaiting()

    if i < strLen:                  # If not all chars have arrived, wait a little
        time.sleep(0.05)
        i = ser.inWaiting()        
    if i < strLen:                  # Wait a little longer still...
        time.sleep(0.05)
        i = ser.inWaiting()

    if i > 0:                       # If something, read and handle the input
        reading = ser.read(i)
        if reading[0] == 21:                    # \x15 = 21 = NAK
            print('NAK-Resend!', reading[0:2])  # To do: resend functionality
            return reading
        elif reading[0] == 6:                   # 6 = ACK
            return reading                      # Return the (hopefully) good data
        else:
            print('Reading error', reading)
            return []
    else:
        print(clear, 'Nothing received') #, end='', flush=True)
        return []

def getInlet():
    if GPIO.input(5): inlet = 6     # testgas
    elif GPIO.input(6): inlet = 3   # 5 meter (taknocken)
    elif GPIO.input(13): inlet = 2  # 30 meter
    elif GPIO.input(19): inlet = 1  # 70 meter
    elif GPIO.input(26): inlet = 0  # 150 meter
    else: inlet = 9          # no valid inlet is found
    return inlet            # note: inlet is local here

def test(cmdStr):
    #ser.write(serial.to_bytes([2, 48, 49, 82, 68, 51, 3, 37]))
    sendNOxCommand(cmdStr)
    time.sleep(0.1)
    i = ser.inWaiting()
    reading = ser.read(i)
    str(binascii.b2a_hex(reading))
    print(reading)

try:
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    ser.bytesize = 7        # check with current instrument! Could also be 7.
    print(ser.name)         # print which port is used

    clear = '\b' * 17   # Backspace (only works in terminal window, not in IDLE shell)
    n = 1       # Total number of meas
    fel = 0     # Read error counter
    avg = 0     # Average accumulator
    nmeas = 0   # Average counter
    inlet = 9   # Inlet selector: 0=150m, 1=70m, 2=30m, 3=5m, 6=T-g (test gas), 9=error
    data = []   # Data series
    write_header = False
    write_header_M = False
    newday = False                  # To avoid printing final text at new day

    GPIO.setmode(GPIO.BCM)      # Set connected pins as inputs
    GPIO.setup(5, GPIO.IN)
    GPIO.setup(6, GPIO.IN)
    GPIO.setup(13, GPIO.IN)
    GPIO.setup(19, GPIO.IN)
    GPIO.setup(26, GPIO.IN)

    GPIO.setup(24, GPIO.OUT)    # Purging pump for 5 m line.
    GPIO.output(24, GPIO.LOW)   # Start w pump off

    while True:
        if not os.path.isfile(saveFile):        # Write headers on new files
            write_header = True
##        else:
##            write_header = False
        if not os.path.isfile(saveFileM):
            write_header_M = True
##        else:
##            write_header_M = False

        with open(saveFile, 'a', newline='') as csvfile, open(saveFileM, 'a', newline='') as csvfileM:
            if write_header:
                csvfile.write('DateTime, NO, NO2, NOx, inlet\r\n')
                write_header = False
            if write_header_M:
                csvfileM.write('DateTime, Instr stat d, Instr stat s, Error code, Warning code, Sample pressure, Reactor pressure, Instr amb temp, PMT temp, Reactor temp, Converter temp, Instr flow, Instr range\r\n')
                write_header_M = False
            try:
                
                while True:
                    if lastdate != datetime.datetime.now().strftime('%d'):
                        lastdate = datetime.datetime.now().strftime('%d')
                        # Construct new savefile(s)
                        saveFile = saveFileStub + '_' + datetime.datetime.now().strftime('%Y-%m-%d') + '.csv'
                        if day_file_instr_data == True:
                            saveFile = saveFileStub + '_' + datetime.datetime.now().strftime('%Y-%m-%d') + '.csv'
                        break                               # EXIT the with clause and open the new savefile instead!
                        
                    time.sleep(sample_time - 0.3)           # Wait some time until next reading
                    while "{:0.1f}".format(time.time() % sample_time) != '0.0':   # Poll the time - sample_time from ini file
                        # - match w other loggers so that they do not poll for the same time!
                        time.sleep(0.01)
                    try:
                        conc = getNOxData()                 # Read and save
                        inlet = getInlet()                  # Get what inlet is measured
                        n += 1

                        t = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                        s = int(t[14:16]) * 60 + int(t[17:19])      # Number of seconds on the hour
                        
                        printstr = "{}, {}, {}, {}, {}".format(t, conc[0], conc[1], conc[2], inlet)
                        print(printstr)
                        printstr += '\r\n'                          # Add rowendings
                        csvfile.write(printstr)

                        if s % sample_time_M == 0:                  # Test the time for sample_time_M
                            data = getNOxinstrumentData()           # Instrument parameters
                            printstr = '{}, {}'.format(t, data)
                            print(printstr)
                            printstr += '\r\n'                      # Add rowendings
                            csvfileM.write(printstr)                # Save instrument data in a separate file
                    except:
                        print("Unexpected error:", sys.exc_info()[0])

                    avg = 0                             # Cleanup
                    nmeas = 0
                    
            except KeyboardInterrupt:   # Press ctrl-c
                print("")
                print('-----------------------')
                print("Antal fel: {:12d}".format(fel))
                print("Antal läsningar: {:6d}".format(n))
                print("Andel fel: {:12.3f}".format(fel/n))
                break                   # Break out of the outer while loop

except Exception as e:
    print ("Error on opening serial port or other: " + str(e))
    exit()
finally:
    ser.close()             # close port
    GPIO.cleanup()          # cleanup all GPIO
    print('Ser port closed.')
