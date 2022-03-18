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

def SendAttachFile():
        msg = MIMEMultipart()
        msg['From'] = fromaddr
        msg['To'] = toaddr
        msg['Subject'] = "iCAR Health Data"
        body = "Attachment Sheet containes => Engine Vibration Data, Temprature Data, Presure sensor data and RPM of Engine"
        msg.attach(MIMEText(body, 'plain'))
        p = path + 'ECU.xlsx'
        print(p)
        attachment = open(p, "rb")
        part = MIMEBase('application', 'octet-stream')
        part.set_payload((attachment).read())
        encoders.encode_base64(part)
        part.add_header('Content-Disposition', "attachment; filename= %s" % p)
        msg.attach(part)
        server = smtplib.SMTP('smtp.gmail.com', 587)
        server.starttls()
        server.login(fromaddr, passwordmail)
        text = msg.as_string()
        server.sendmail(fromaddr, toaddr, text)
        server.quit()

fromaddr = "mkparmarkec@gmail.com"
toaddr = "tushuni@gmail.com"
passwordmail = "Evaparmar@12"

print("iCAR+100 for Sending Car Health Data..........................")

path = '/home/pi/Downloads/iCAR/'

msg = MIMEMultipart()
msg['From'] = fromaddr
msg['To'] = toaddr
msg['Subject'] = "10 FILES ATTACHED AND SENT BY AUTOMATIC PROGRAM"

body = "ATTACHED FROM VMWARE VIRTUAL MACHINE Because program start form bigining"

msg.attach(MIMEText(body, 'plain'))

SendAttachFile()
