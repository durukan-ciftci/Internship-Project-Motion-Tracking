import serial
import threading
import time
from gpiozero import LED
import queue
from multiprocessing import Process, Queue
import cv2 as cv
from pypylon import pylon
import numpy as np

SLAVE3_ADRESS = 0x03


rx_buffer = bytearray()
is_started = False
byte_counter = 0
is_error = False
is_adress_me = False
is_response = False

S_command_H = 0
S_command_L = 0
S_data_H = 0
S_data_L = 0
S_adress = 0
S_checksum = 0

x_pos = 0
y_pos = 0



def calculate_checksum(S_adress, S_command_H, S_command_L, S_data_H, S_data_L):
    checksum_val = (S_adress + S_command_H + S_command_L + S_data_H + S_data_L) % 256 
    return checksum_val

def check_sum(S_adress, S_command_H, S_command_L, S_data_H, S_data_L, S_checksum):
    if(S_checksum == (S_adress + S_command_H + S_command_L + S_data_H + S_data_L) % 256 ):
        return True
    else:
        return False
def S_Camera_x_position_response(slave_adress, position_data):
    data_L = int(position_data) % 256
    data_H = int(position_data) // 256
    
    tx_buffer = bytes([0xFF, slave_adress, 0x00, 0x47, data_H, data_L, calculate_checksum(slave_adress, 0x00, 0x47, data_H, data_L)])
    print(f"[queue] {tx_buffer}")
    tx_queue.put(tx_buffer)

def S_Camera_y_position_response(slave_adress, position_data):
    data_L = int(position_data) % 256
    data_H = int(position_data) // 256
    tx_buffer = bytes([0xFF, slave_adress, 0x00, 0x67, data_H, data_L, calculate_checksum(slave_adress, 0x00, 0x67, data_H, data_L)])
    print(f"[queue] {tx_buffer}")
    tx_queue.put(tx_buffer)

#setup serial port
ser = serial.Serial(
    port = '/dev/ttyAMA0',
    baudrate  = 115200,
    timeout = 0.5
)

tx_queue = queue.Queue()

en_pin = LED(26)

def uart_transmit():
    global is_response
    while True:

        if is_response:
            message = tx_queue.get()
            en_pin.on()
            ser.write(message)
            ser.flush()
            print(f"[UART Sent]", message.hex())
            en_pin.off()
        time.sleep(0.01)

def uart_receive(): # thread for non-blocking serial receive
    global rx_buffer
    global byte_counter

    global S_command_H
    global S_command_L
    global S_data_H
    global S_data_L
    global S_adress

    global is_started
    global is_error
    global is_response
    global is_adress_me

    global x_pos 
    global y_pos 

    while True:
        if ser.in_waiting:
            byte = ser.read(1)
            if not byte:
                continue

            elif ((not is_error) & (not is_started) & (byte[0] == 0xFF)):
                rx_buffer += byte
                is_started = True
                byte_counter += 1
            
            elif((not is_error) & (is_started) & (byte_counter == 1) & (byte[0] == 0x03)):
                rx_buffer += byte
                S_adress = byte[0]
                is_adress_me = True
                byte_counter += 1

            elif((not is_error) & (is_started)  & (not is_adress_me) & (byte_counter <= 6)): # adress does not belong to the slave
                #rx_buffer += byte
                if (byte_counter == 6) :
                    #print("wrong adress")
                    rx_buffer.clear()
                    is_started = False
                    byte_counter = 0
                    is_adress_me = False
                else:
                    byte_counter += 1

            elif((not is_error) & (is_started ) & (is_adress_me) & (byte_counter < 6)): # adress does belong to the slave
                rx_buffer += byte
                match byte_counter:
                    case 2:
                        S_command_H = byte[0]
                        
                    case 3:
                        S_command_L = byte[0]
                        
                    case 4:
                        S_data_H = byte[0]
                        
                    case 5:
                        S_data_L = byte[0]

                    case _:
                        print("default case")
                
                byte_counter += 1
                           
            elif((not is_error) & (is_started) & (is_adress_me) & (byte_counter == 6)):
                rx_buffer += byte
                S_checksum = byte[0]
                if (check_sum(S_adress, S_command_H, S_command_L, S_data_H, S_data_L, S_checksum)): 
                    match(S_command_L):
                        case 0x4F:
                            is_response = True
                            S_Camera_x_position_response(SLAVE3_ADRESS, x_pos)
                            print(f"Reception completed",S_adress, S_command_H, S_command_L, S_data_H, S_data_L)
                        case 0x6F:
                            is_response = True
                            S_Camera_y_position_response(SLAVE3_ADRESS, y_pos)
                            print(f"Reception completed",S_adress, S_command_H, S_command_L, S_data_H, S_data_L)
                        case _:
                            print('command not defined')
                    is_error = False
                else:
                    print(f"ERROR IN TRANSMISSION")
                    is_error = True
                rx_buffer.clear()
                is_started = False
                byte_counter = 0
                is_adress_me = False
            
            else:
                is_error = True 
                rx_buffer.clear()
                is_started = False
                is_error = False
                byte_counter = 0

def cam_image_processing():

    def draw_cross(x,y,length):
        x3 = x1 = int(x - length)
        y4 = y1 = int(y - length)
        x4 = x2 = int(x + length)
        y3 = y2 = int(y + length)
        cv.line(img_detected, (x1,y1), (x2,y2), (0,255,0), 2)
        cv.line(img_detected, (x3,y3), (x4,y4), (0,255,0), 2)
        cv.line(img_square_detection, (x1,y1), (x2,y2), (0,255,0), 2)
        cv.line(img_square_detection, (x3,y3), (x4,y4), (0,255,0), 2)
    
    x_old_pos = 0xAA
    y_old_pos = 0xBB
    global x_pos 
    global y_pos
    prev_frame = None

    x_values_detected = []
    y_values_detected = []

    camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
    camera.Open()
    camera.StartGrabbing()

    converter = pylon.ImageFormatConverter()
    converter.OutputPixelFormat = pylon.PixelType_Mono8
    converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned


    while camera.IsGrabbing():
        grab_result = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        if grab_result.GrabSucceeded():
            image = converter.Convert(grab_result)
            img = image.GetArray(); # grayscale image is obtained
            img_detected = img.copy()
            img_square_detection = img.copy()
            if prev_frame is not None:
                diff = cv.absdiff(prev_frame, img)
                blur = cv.GaussianBlur(diff, (5,5),0)
                _, thresh = cv.threshold(blur, 80, 255, cv.THRESH_BINARY)
                contours,_ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
                for cnt in contours:
                    if cv.contourArea(cnt) < 2000:
                        continue
                    x, y, w, h = cv.boundingRect(cnt)
                    if y < 1000:
                        cv.rectangle(img_square_detection, (x, y), (x + w, y + h), 255, 2)
                        x_values_detected.append(x+w/2)
                        y_values_detected.append(y+h/2)
                
                if len(x_values_detected):
                    
                    x_values_detected_arr = np.array(x_values_detected)
                    y_values_detected_arr = np.array(y_values_detected)
                    """
                    Another tracking code (not tested)
                    x_temp_pos = x_old_pos
                    y_temp_pos = y_old_pos
                    x_old_pos, y_old_pos = x_pos, y_pos
                    gaussian_distances_arr = (x_values_detected_arr - x_temp_pos)**2 +  (y_values_detected_arr - y_temp_pos)**2
                    track_detected_num = gaussian_distances_arr.argmin()
                    x_pos, y_pos = x_values_detected_arr[track_detected_num], y_values_detected_arr[track_detected_num]
                    """
                    x_pos = int(np.mean(x_values_detected_arr)) # average of positions
                    y_pos = int(np.mean(y_values_detected_arr)) # average of positions
                    x_values_detected =[] 
                    y_values_detected = []
                
                
                draw_cross(x_pos,y_pos,20)
                #cv.imshow("all detected",img_square_detection) to see all of the motion detected
                cv.imshow( "Camera Motion Detection", img_detected)
            prev_frame = img
        if(cv.waitKey(1) & 0xFF == ord("q")):
            break
        grab_result.Release()

    camera.StopGrabbing()
    camera.Close()
    cv.destroyAllWindows()

cam_thread = threading.Thread(target = cam_image_processing, daemon = True)
cam_thread.start()

rx_thread = threading.Thread(target = uart_receive, daemon = True)
rx_thread.start()

tx_thread = threading.Thread(target = uart_transmit, daemon = True)
tx_thread.start()

try:
    while True:

        time.sleep(5)

except KeyboardInterrupt:
    print("Exiting...")
    ser.close()