# Import the serial and csv libraries
import serial
import csv
import datetime

# Define the serial object
serial_obj = serial.Serial()


# Set baudrate, Communication port and Open the serial port for listening
serial_obj.baudrate = 115200
serial_obj.port = "COM9"
serial_obj.open() 

data_points = 0
required_nos = 5000



file_name = 'Dataset/Nov_18_Surya_1.csv'




with open(file_name, 'a', newline='') as file:
    writer = csv.writer(file)
    #writer.writerow(["mic1","mic2","mic3","mic4","mic5","angle"])
    while data_points<required_nos:	
        if serial_obj.in_waiting:
            packet = serial_obj.readline()                 
            decoded = packet.decode('utf')                     
            data = decoded[:-2].split(',')                         
            print(data)                               
            data_points = data_points+1 
            writer.writerow(data) 
					
    file.close()