import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import cv2
import numpy as np
import serial
import time
import threading

# Initialize variables
angles = [90, 90, 90]
gripperStates = [0, 0]

# Variables for autofind function
x_mid = None
y_mid = None
center_x = 0
auto_find_running = False
# Initialize camera
cap = cv2.VideoCapture(0)
SERIAL_PORT = 'COM3'
BAUD_RATE = 9600
READ_TIMEOUT = 1

# Initialize serial connection 
try:
    ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=READ_TIMEOUT)
    print(f"Connected to {SERIAL_PORT}")
except serial.SerialException as e:
    print(f"Failed to connect to {SERIAL_PORT}: {e}")
    ser = None
#main window
root = tk.Tk()
root.title("Robotic Arm Control Panel")
root.geometry("800x700")
root.configure(bg="#0a0a1a")
# Configure styles
style = ttk.Style()
style.theme_use("clam")
style.configure("TFrame", background="#0a0a1a")
style.configure("TLabel", background="#0a0a1a", foreground="#33ccff", font=("Courier", 12, "bold"))
style.configure("TScale", background="#0a0a1a", foreground="#1f1f3a")
style.configure("TButton", font=("Courier", 12, "bold"), foreground="white", background="#f58916", padding=6)

canvas = tk.Canvas(root, width=800, height=700, bg="#0a0a1a", highlightthickness=0)
canvas.place(x=0, y=0)

import random

# Add stars to background
for _ in range(150):
    x, y = random.randint(0, 800), random.randint(0, 700)
    size = random.randint(1, 2)
    canvas.create_oval(x, y, x+size, y+size, fill="white", outline="")
  
frame = ttk.Frame(root, style="TFrame")
frame.place(x=50, y=20)

servo_count = 3
sliders = []
angle_labels = []


for i in range(servo_count):
    label = ttk.Label(frame, text=f"Servo {i + 1} Angle: ")
    label.grid(row=i, column=0, sticky="w", pady=5)


    slider = ttk.Scale(frame, from_=0, to=180, orient=tk.HORIZONTAL, length=400)
    slider.set(90)
    slider.grid(row=i, column=1, pady=5)
    sliders.append(slider)
   
    angle_label = ttk.Label(frame, text=f"Current Angle: {angles[i]}")
    angle_label.grid(row=i, column=2, sticky="w", pady=5)
    angle_labels.append(angle_label)


# Global variable to store the photo reference
current_photo = None

def set_open():
    global gripperStates
    gripperStates[1] = 1


def set_close():
    global gripperStates
    gripperStates[1] = -1


def set_CCW():
    global gripperStates
    gripperStates[0] = 1


def set_CW():
    global gripperStates
    gripperStates[0] = -1


def update_camera():
    global current_photo, x_mid, y_mid, center_x


    ret, frame = cap.read()
    if not ret:
        root.after(30, update_camera)
        return


    frame_height, frame_width = frame.shape[:2]
    center_x = frame_width // 2


    # Convert to grayscale 
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


    # Threshold for black
    _, mask = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY_INV)

    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    x_mid = None
    y_mid = None

    if contours:
        # Find largest contour 
        largest = max(contours, key=cv2.contourArea)
       
        if cv2.contourArea(largest) > 100: 
            M = cv2.moments(largest)
            if M["m00"] != 0:
                x_mid = int(M["m10"] / M["m00"])
                y_mid = int(M["m01"] / M["m00"])
                cv2.circle(frame, (x_mid, y_mid), 6, (0, 0, 255), -1)  # Red dot

            x, y, w, h = cv2.boundingRect(largest)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)

    cv2.line(frame, (center_x, frame_height), (center_x, 0), (0, 255, 0), 2)

    # Convert to RGB and display
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    resized_frame = cv2.resize(frame_rgb, (360, 270))
    img = Image.fromarray(resized_frame)
    current_photo = ImageTk.PhotoImage(image=img)
    camera_label.configure(image=current_photo)

    for i, angle_label in enumerate(angle_labels):
        current_angle = int(sliders[i].get())
        angles[i] = current_angle
        angle_label.configure(text=f"Current Angle: {current_angle}")
    root.after(30, update_camera)


# Create camera display label
camera_label = tk.Label(root, bg="#0a0a1a")
camera_label.place(x=200, y=320)
def send_angles():
    if ser is None:
        print("Serial connection not available")
        return
   
    try:
        current_angles = [int(slider.get()) for slider in sliders]
        current_angles += gripperStates
        print(f"Sending angles: {current_angles}")
        ser.write(f"{','.join(map(str, current_angles))}\n".encode())
    except serial.SerialException as e:
        print(f"Serial communication error: {e}")


def send_specific_angles(angle_list):
    """send specific angles to the robotic arm"""
    if ser is None:
        print("Serial connection not available")
        return False
   
    try:
        ser.write(f"{','.join(map(str, angle_list))}\n".encode())
        return True
    except serial.SerialException as e:
        print(f"Serial communication error: {e}")
        return False


def autoFind():
    global x_mid, center_x, auto_find_running
   
    if auto_find_running:
        print("AutoFind already running")
        return
   
    auto_button.configure(text="Stop AutoFind", command=stop_auto_find)
    auto_find_running = True
   
    thread = threading.Thread(target=autoFind_worker)
    thread.daemon = True
    thread.start()


def stop_auto_find():
    global auto_find_running
    auto_find_running = False
    auto_button.configure(text="Automated Find", command=autoFind)
    print("AutoFind stopped")

def autoFind_worker():
    global x_mid, center_x, auto_find_running
   
    print("Starting AutoFind...")
   
    initial_angles = [114, 90, 90, 1, 1]  # [servo1, servo2, servo3, wrist_rotation, gripper]
    send_specific_angles(initial_angles)
    time.sleep(3)
    if not send_specific_angles(initial_angles):
        stop_auto_find()
        return


    print(f"Moving to initial search position: {initial_angles}")
    time.sleep(5)  # Give time for arm to move
    initial_angles = [114, 90, 90, 0, 0]
    send_specific_angles(initial_angles)
    # Wait for object detection with timeout
    timeout_counter = 0
    max_timeout = 50  # 5 seconds at 0.1s intervals
   
    while x_mid is None and auto_find_running and timeout_counter < max_timeout:
        print(f"Waiting for object detection... ({timeout_counter}/{max_timeout})")
        time.sleep(5)
        timeout_counter += 1
   
    if not auto_find_running:
        return
   
    if x_mid is None:
        print("No object detected within timeout period")
        stop_auto_find()
        return
   
    print(f"Object detected at x = {x_mid}, center = {center_x}")
   
    # Centering algorithm
    base_angle = 114
    centered = False
    max_attempts = 100
    attempt_count = 0
    tolerance = 10  # Pixels tolerance 
   
    while not centered and auto_find_running and attempt_count < max_attempts:
        if x_mid is None:
            print("Lost object during centering")
            time.sleep(3)
            attempt_count += 1
            continue
       
        if abs(x_mid - center_x) <= tolerance:
            print(f"Object centered! x = {x_mid}, center = {center_x}")
            centered = True
            break
       
        # Calculate adjustment
        error = x_mid - center_x
        adjustment = error * 0.05  # Proportional control
        base_angle -= adjustment 
      
        base_angle = max(90, min(160, base_angle))
       
        current_angles = [int(base_angle), 90, 90, 0, 0]
       
        if send_specific_angles(current_angles):
            print(f"Adjusting base angle to {base_angle:.1f} (error: {error})")
            time.sleep(5)  
        else:
            break
       
        attempt_count += 1
   
    if centered:
        print("Object successfully centered!")
        fix = 7


        '''
        final_angles = [int(base_angle) - fix, 90, 90, 1, 1]  
        send_specific_angles(final_angles)
        time.sleep(5)
        '''
        #set arm to baseangle - fix + 30, 29 shoulder, 29 elbow
        targetBase = int(base_angle)
        stepupBase = targetBase - fix + 30


        final_angles = [stepupBase, 90, 25, -1, 1]
        send_specific_angles(final_angles)
        time.sleep(5)


        for i in range(90, 25, -5):
            final_angles =[stepupBase, i, 25, -1, 1]
            send_specific_angles(final_angles)
            time.sleep(1)
        final_angles =[stepupBase, 29, 25, -1, 0]
        send_specific_angles(final_angles)
        time.sleep(1)


        final_angles= [targetBase, 29, 25, -1, 1]
        send_specific_angles(final_angles)
        time.sleep(1)

        #set arm to baseangle - fix, 29 shoulder, 45 elbow for grabbing
        final_angles= [targetBase, 29, 45, -1, 1]
        send_specific_angles(final_angles)
        time.sleep(20)


        print("In place to grab object")
        final_angles = [targetBase, 29, 45, -1, -1]  # Close gripper
        send_specific_angles(final_angles)
        time.sleep(10)
        print("Gripper closed")
        final_angles = [159, 80, 60, -1, -1]
        send_specific_angles(final_angles)
        time.sleep(5)
        final_angles = [159, 80, 60, -1, 1]
        send_specific_angles(final_angles)
        time.sleep(2)
        final_angles = [159, 80, 60, 0, 0]
        send_specific_angles(final_angles)
    else:
        print(f"Failed to center object after {max_attempts} attempts")
   
    # Reset button text
    root.after(0, stop_auto_find)

button_frame = ttk.Frame(frame, style="TFrame")
button_frame.grid(row=servo_count, column=0, columnspan=3, pady=20)

# Wrist and gripper control buttons 
gripper_open = ttk.Button(button_frame, text="Open Gripper", command=set_open, style="TButton")
gripper_open.grid(row=0, column=0, padx=5, pady=2)

gripper_close = ttk.Button(button_frame, text="Close Gripper", command=set_close, style="TButton")
gripper_close.grid(row=0, column=1, padx=5, pady=2)

wrist_CCW = ttk.Button(button_frame, text="Rotate CCW", command=set_CCW, style="TButton")
wrist_CCW.grid(row=0, column=2, padx=5, pady=2)

wrist_CW = ttk.Button(button_frame, text="Rotate CW", command=set_CW, style="TButton")
wrist_CW.grid(row=0, column=3, padx=5, pady=2)

send_button = ttk.Button(button_frame, text="Send Angles", command=send_angles, style="TButton")
send_button.grid(row=1, column=0, padx=5, pady=2)

auto_button = ttk.Button(button_frame, text="Automated Find", command=autoFind, style="TButton")
auto_button.grid(row=1, column=1, padx=5, pady=2)  

# Status label for feedback
status_label = ttk.Label(button_frame, text="Ready", foreground="#00ff00")
status_label.grid(row=1, column=2, columnspan=2, padx=5, pady=2)

# Start camera update loop
update_camera()

def on_closing():
    global auto_find_running
    auto_find_running = False
    cap.release()
    cv2.destroyAllWindows()
    if ser:
        ser.close()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_closing)

# Start the GUI
root.mainloop()
