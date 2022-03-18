import can
import time
import os
import xlsxwriter

array = []
i = 0
k=1
myarray = []
file_name = 'ECU'
print('\n\rCAN Rx test')
print('Bring up CAN0....')
os.system("sudo /sbin/ip link set can0 up type can bitrate 500000")
time.sleep(0.1)	

path = '/home/pi/Downloads/iCAR/'

try:
	bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
except OSError:
	print('Cannot find PiCAN board.')
	exit()
	
print('Ready')

def longitude():
        path1 = '/home/pi/Downloads/iCAR/'
        os.chdir(path1)
        os.system("sudo python2 GPS_READ_27.py")

try:
        while True:
                workbook = xlsxwriter.Workbook(file_name +'.xlsx', {'tmpdir': '/home/pi/Downloads/iCAR/'})
                worksheet = workbook.add_worksheet()
                
                worksheet.write(0, 0, 'Message Id')
                worksheet.write(0, 1, 'XL')
                worksheet.write(0, 2, 'XH')
                worksheet.write(0, 3, 'YL')
                worksheet.write(0, 4, 'YH')
                worksheet.write(0, 5, 'ZL')
                worksheet.write(0, 6, 'ZH')
                worksheet.write(0, 7, 'Temp')
                worksheet.write(0, 8, 'RPM/Pressure')
                                
                for j in range(1, 109):
                        message = bus.recv()
                        worksheet.write(j, 0, message.arbitration_id)
                        for i in range(message.dlc ):
                                array.append(message.data[i])
                                worksheet.write(j, i+1, message.data[i])

                workbook.close()
                os.chdir(path)
                os.system("sudo python2 SMTP_File_send.py")
                print("Logging File Sent..............")
                #longitude()
except KeyboardInterrupt:
	#Catch keyboard interrupt
	os.system("sudo /sbin/ip link set can0 down")
	print('\n\rKeyboard interrtupt')


