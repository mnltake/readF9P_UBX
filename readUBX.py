#RELPOSNED and POSLLH

import serial
import pprint
HEADER = 6
RELPOSNED = b'\x3c'
lenRELPOSNED = HEADER + 64 +2
POSLLH = b'\x02'
lenPOSLLH = HEADER + 28 + 2
PVT =b'\x07'
lenPVT = HEADER + 92 +2

def readUBX():
    msg = dict()
    buffsize = lenRELPOSNED + lenPVT #172
    with serial.Serial('COM17', 115200, timeout=1) as ser:
        buffer =[]
        j=0   
        for b in range(buffsize):
            buffer.append(ser.read())
        #print(buffer)
        while j < buffsize : 
            i = 0
            payloadlength = 0
            ackPacket=[b'\xB5',b'\x62',b'\x01',b'\x00',b'\x00',b'\x00']
            while i < payloadlength +8:              
                if j < buffsize :
                    incoming_byte = buffer[j]   
                    j += 1
                else :
                    break
                if (i < 3) and (incoming_byte == ackPacket[i]):
                    i += 1
                elif i == 3:
                    ackPacket[i]=incoming_byte
                    i += 1              
                elif i == 4 :
                    ackPacket[i]=incoming_byte
                    i += 1
                elif i == 5 :
                    ackPacket[i]=incoming_byte        
                    payloadlength = int.from_bytes(ackPacket[4]+ackPacket[5], byteorder='little',signed=False) 
                    i += 1
                elif (i > 5) :
                    ackPacket.append(incoming_byte)
                    i += 1

            
            if checksum(ackPacket,payloadlength) :
                if ackPacket[3] == RELPOSNED:
                    msg.update(perseNED(ackPacket))
                elif ackPacket[3] == POSLLH:
                    msg.update(perseLLH(ackPacket))
                elif ackPacket[3] == PVT:
                    msg.update(persePVT(ackPacket))
    return msg

def checksum(ackPacket,payloadlength ):
    CK_A =0
    CK_B =0
    for i in range(2, payloadlength+6):
        CK_A = CK_A + int.from_bytes(ackPacket[i], byteorder='little',signed=False) 
        CK_B = CK_B +CK_A
    CK_A &=0xff
    CK_B &=0xff
    if (CK_A ==  int.from_bytes(ackPacket[-2], byteorder='little',signed=False)) and (CK_B ==  int.from_bytes(ackPacket[-1], byteorder='little',signed=False)):
        #print("ACK Received")
        return True
    else :
        print("ACK Checksum Failure:")  
        return False

def perseNED(ackPacket):
    posned = dict()
    #relPosN
    byteoffset =8 +HEADER
    bytevalue =  ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    posned["N"] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    posned["NH"] = int.from_bytes(ackPacket[32 + HEADER], byteorder='little',signed=True) 
    #print("N:%0.2f cm" %posned["N"]  )
    #relPosE
    byteoffset =12 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    posned["E"] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    posned["EH"] = int.from_bytes(ackPacket[33 + HEADER], byteorder='little',signed=True) 
    #print("E:%0.2f cm" %posned["E"]  )
    #relPosD
    byteoffset =16 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    posned["D"] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    posned["DH"] = int.from_bytes(ackPacket[33 + HEADER], byteorder='little',signed=True)     #print("D:%0.2f cm" %posned["D"]  )
    #Carrier solution status
    flags = int.from_bytes(ackPacket[60 + HEADER], byteorder='little',signed=True) 
    posned["gnssFixOk"] =  flags  & (1 << 0) #gnssFixOK 
    posned["carrSoln"] =  (flags   & (0b11 <<3)) >> 3 #carrSoln0:no carrier 1:float 2:fix
    #print("gnssFixOk:%d" %posned["gnssFixOk"])
    #print("carrSoln:%d" %posned["carrSoln"])
    #GPS time
    byteoffset =4 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    posned["iTow"] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    #print("iTow:%0.1f" %float(posned["iTow"]/1000))
    #relPosLength
    byteoffset =20 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    posned["length"] = int.from_bytes(bytevalue, byteorder='little',signed=False) 
    posned["lengthH"] = int.from_bytes(ackPacket[35 + HEADER], byteorder='little',signed=True) 
    #print("length:%0.1f cm" %float(posned["length"]))
    #relPosHeading
    byteoffset =24 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    posned["heading"] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    #print("heading:%f deg" %float(posned["heading"]/100000))
    
    return posned

def perseLLH(ackPacket):
    posllh=dict()
    #PosLon
    byteoffset = 4 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    posllh["Lon"] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    #print("Lon:%f " %float(posllh["Lon"] /10000000) )
    #PosLat
    byteoffset =8 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    posllh["Lat"] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    #print("LAT:%f " %float(posllh["Lat"] /10000000) )

    #posHeight
    byteoffset =12 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    posllh["Height"] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    #print("Height:%.4f m" %float(posllh["Height"]/100000)  )

    #Height above mean sea level
    byteoffset =16 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    posllh["hMSL"] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    #print("hMSL :%.4f m" %float(posllh["hMSL"]/100000)  )

    return posllh

def persePVT(ackPacket):
    pospvt=dict()
    #Year
    byteoffset = 4 +HEADER
    bytevalue = ackPacket[byteoffset] 
    bytevalue  +=  ackPacket[byteoffset+1] 
    pospvt["year"] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    #print("year:%d " %pospvt["year"] )
    #month day hour min sec
    byteoffset =6 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for key in ("month", "day", "hour", "min", "sec"):
        i =0
        bytevalue  =  ackPacket[byteoffset+i] 
        pospvt[key] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
        i +=1
    #print("MDhms:%d/%d-%d:%d:%d " %(pospvt["month"],pospvt["day"],pospvt["hour",pospvt["min"],pospvt["sec"] ))

    #PosLon
    byteoffset = 24 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    pospvt["Lon"] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    #print("Lon:%f " %float(pospvt["Lon"] /10000000) )
    #PosLat
    byteoffset =28 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    pospvt["Lat"] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    #print("Lat:%f " %float(pospvt["Lat"] /10000000) )

    #posHeight
    byteoffset =32 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    pospvt["Height"] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    #print("Height:%.4f m" %float(pospvt["Heght"]/1000)  )

    #Height above mean sea level
    byteoffset =36 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    pospvt["hMSL"] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    #print("hMSL :%.4f m" %float(pospvt["hMSL"]/1000)  )

    #Ground Speed
    byteoffset =60 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    pospvt["gSpeed"] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    #print("gSpeed :%.4f m/s" %float(pospvt"gSpeed"]/1000)  )
    return pospvt

while 1:
    ubxmsg=readUBX()
    pprint.pprint(ubxmsg)

    """
    buffsize = lenRELPOSNED + lenPVT #172
    with serial.Serial('COM17', 115200, timeout=1) as ser:
        readbytes =[]
          
        for i in range(buffsize):
            readbytes.append(ser.read())
        #print(readbytes)
        ubxmsg=readUBX(readbytes)
        pprint.pprint(ubxmsg)
    """


