#RELPOSNED and POSLLH

import serial
HEADER = 6
RELPOSNED = b'\x3c'
lenRELPOSNED = HEADER + 64 +2
POSLLH = b'\x02'
lenPOSLLH = HEADER + 28 + 2
PVT =b'\x07'
lenPVT = HEADER + 92 +2

def readUBX():
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
                elif (i > 5):
                    ackPacket.append(incoming_byte)
                    i += 1
            if checksum(ackPacket,payloadlength) :
                if ackPacket[3] == RELPOSNED:
                    perseNED(ackPacket)
                elif ackPacket[3] == POSLLH:
                    perseLLH(ackPacket)
                elif ackPacket[3] == PVT:
                    persePVT(ackPacket)

def checksum(ackPacket,payloadlength ):
    CK_A =0
    CK_B =0
    for i in range(2, payloadlength+6):
        CK_A = CK_A + int.from_bytes(ackPacket[i], byteorder='little',signed=False) 
        CK_B = CK_B +CK_A
    CK_A &=0xff
    CK_B &=0xff
    if (CK_A ==  int.from_bytes(ackPacket[-2], byteorder='little',signed=False)) and (CK_B ==  int.from_bytes(ackPacket[-1], byteorder='little',signed=False)):
        print("ACK Received")
        return True
    else :
        print("ACK Checksum Failure:")  
        return False

def perseNED(ackPacket):
    posned = [0]*8
    #relPosN
    byteoffset =8 +HEADER
    bytevalue =  ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    posned[0] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    posned[0] += (int.from_bytes(ackPacket[32 + HEADER], byteorder='little',signed=True) )/100
    print("N:%0.2f cm" %posned[0]  )
    #relPosE
    byteoffset =12 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    posned[1] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    posned[1] += (int.from_bytes(ackPacket[33 + HEADER], byteorder='little',signed=True) )/100
    print("E:%0.2f cm" %posned[1]  )
    #relPosD
    byteoffset =16 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    posned[2] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    posned[2] += (int.from_bytes(ackPacket[33 + HEADER], byteorder='little',signed=True) )/100
    print("D:%0.2f cm" %posned[2]  )
    #Carrier solution status
    flags = int.from_bytes(ackPacket[60 + HEADER], byteorder='little',signed=True) 
    posned[3] =  flags  & (1 << 0) #gnssFixOK 
    posned[4] =  (flags   & (0b11 <<3)) >> 3 #carrSoln0:no carrier 1:float 2:fix
    print("gnssFixOk:%d" %posned[3])
    print("carrSoln:%d" %posned[4])
    #GPS time
    byteoffset =4 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    posned[5] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    print("iTow:%0.1f" %float(posned[5]/1000))
    #relPosLength
    byteoffset =20 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    posned[6] = int.from_bytes(bytevalue, byteorder='little',signed=False) 
    posned[6] += (int.from_bytes(ackPacket[35 + HEADER], byteorder='little',signed=True) ) /100
    print("length:%0.1f cm" %float(posned[6]))
    #relPosHeading
    byteoffset =24 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    posned[7] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    print("heading:%f deg" %float(posned[7]/100000))
    
    return posned

def perseLLH(ackPacket):
    posllh=[0]*4
    #PosLon
    byteoffset = 4 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    posllh[0] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    print("LON:%f " %float(posllh[0] /10000000) )
    #PosLat
    byteoffset =8 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    posllh[1] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    print("LAT:%f " %float(posllh[1] /10000000) )

    #posHeight
    byteoffset =12 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    posllh[2] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    print("Height:%.4f m" %float(posllh[2]/100000)  )

    #Height above mean sea level
    byteoffset =16 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    posllh[3] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    print("hMSL :%.4f m" %float(posllh[3]/100000)  )

    return posllh
def persePVT(ackPacket):
    pospvt=[0]*11
    #Year
    byteoffset = 4 +HEADER
    bytevalue = ackPacket[byteoffset] 
    bytevalue  +=  ackPacket[byteoffset+1] 
    pospvt[0] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    print("year:%d " %pospvt[0] )
    #month day hour min sec
    byteoffset =6 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(5):
        bytevalue  =  ackPacket[byteoffset+i] 
        pospvt[1+i] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    print("MDhms:%d/%d-%d:%d:%d " %(pospvt[1],pospvt[2],pospvt[3],pospvt[4],pospvt[5] ))

    #PosLon
    byteoffset = 24 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    pospvt[6] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    print("LON:%f " %float(pospvt[6] /10000000) )
    #PosLat
    byteoffset =28 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    pospvt[7] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    print("LAT:%f " %float(pospvt[7] /10000000) )

    #posHeight
    byteoffset =32 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    pospvt[8] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    print("Height:%.4f m" %float(pospvt[8]/1000)  )

    #Height above mean sea level
    byteoffset =36 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    pospvt[9] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    print("hMSL :%.4f m" %float(pospvt[9]/1000)  )

    #Ground Speed
    byteoffset =60 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    pospvt[10] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    print("aSpeed :%.4f m/s" %float(pospvt[10]/1000)  )
    return pospvt

while 1:
    readUBX()