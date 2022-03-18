# -*- coding: utf-8 -*-
import serial
import time

import os
import sys
import smtplib
import mimetypes

from optparse import OptionParser

from email import encoders
from email.message import Message
from email.mime.base import MIMEBase
from email.mime.image import MIMEImage
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText

myserial = serial.Serial('/dev/ttyUSB0', 9600)
i=0
arr = []
myarray = []
coordinates_lat = []
coordinates_lon = []

def SendAttachFile():
        body = link
        msg.attach(MIMEText(body, 'plain'))
        part = MIMEBase('application', 'octet-stream')
        encoders.encode_base64(part)
        server = smtplib.SMTP('smtp.gmail.com', 587)
        server.starttls()
        server.login(fromaddr, passwordmail)
        text = msg.as_string()
        server.sendmail(fromaddr, toaddr, text)
        server.quit()

fromaddr = "mkparmarkec@gmail.com"
toaddr = "tushuni@gmail.com"
passwordmail = "Evaparmar@12"

print("Connecting iCAR+100 v1.O for security email ..........................")

msg = MIMEMultipart()
msg['From'] = fromaddr
msg['To'] = toaddr
msg['Subject'] = "Please click this link............."

k=0

while True:
    while (i<30):
        arr.append(myserial.readline())
        i = i+1
    y = len(arr)
    for x in range(0, len(arr)):
        myarray.append(arr[x].split(','))

    for q in range(1, len(myarray)):
        for w in range(0, len(myarray[q])):
            if myarray[q][w] == '$GPRMC':
                validation = myarray[q][2]
                lat = myarray[q][3]
                lon = myarray[q][5]
                coordinates_lat.append(lat)
                coordinates_lon.append(lon)        

    if validation=='A':
        latitude = coordinates_lat[0]
        longitude = coordinates_lon[0]
        L1 = int(float(latitude)/100)

        L2 = float(latitude)%100
        L3 = ((float(latitude)%100)-int(L2))*100

        F_LAT = (((((L3)/60) + L2)/60)+L1)
        
        L4 = int(float(longitude)/100)
        L5 = float(longitude)%100
        L6 = ((float(longitude)%100)-int(L5))*100

        F_LONG = (((((L6)/60) + L5)/60)+L4)

        print(F_LAT)
        print(F_LONG)
        body = str(F_LAT) + "Latitude Coordinates"
        msg.attach(MIMEText(body, 'plain'))
        body = str(F_LONG) + "Longitude Coordinates" 
        msg.attach(MIMEText(body, 'plain'))
        link = "https://www.google.co.in/maps/place/"+str(L1)+"%C2%B0"+str(L2)+"'"+str(L3)+"%22N+"+str(L4)+"%C2%B0"+str(L5)+"'"+str(L6)+"%22E/@" +str(F_LAT)+","+str(F_LONG)
        
        SendAttachFile()
        if k==1:
                break
        else:                
                coordinates_lat = []
                coordinates_lon = []
                myarray = []
                k = k+1
    time.sleep(1)
