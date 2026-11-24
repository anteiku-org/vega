import socket
import tkinter as tk
import time

UDP_IP = "192.168.4.1" # Default ESP32 AP IP
UDP_PORT = 4210

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_data():
    t = throttle.get()
    r = roll.get()
    p = pitch.get()
    y = yaw.get()
    
    # Format: T:1500,R:0,P:0,Y:0
    msg = f"T:{t},R:{r},P:{p},Y:{y}"
    sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))
    
    # Send at 50Hz (every 20ms)
    root.after(20, send_data)

def stop_motors():
    throttle.set(1000)
    roll.set(0)
    pitch.set(0)
    yaw.set(0)

root = tk.Tk()
root.title("Falcon FC UDP Controller")
root.geometry("400x300")

tk.Label(root, text="Throttle (1000-2000)").pack()
throttle = tk.Scale(root, from_=2000, to=1000, orient=tk.VERTICAL)
throttle.set(1000)
throttle.pack()

tk.Label(root, text="Roll (-30 to 30)").pack()
roll = tk.Scale(root, from_=-30, to=30, orient=tk.HORIZONTAL)
roll.set(0)
roll.pack()

tk.Label(root, text="Pitch (-30 to 30)").pack()
pitch = tk.Scale(root, from_=-30, to=30, orient=tk.HORIZONTAL)
pitch.set(0)
pitch.pack()

tk.Label(root, text="Yaw (-45 to 45)").pack()
yaw = tk.Scale(root, from_=-45, to=45, orient=tk.HORIZONTAL)
yaw.set(0)
yaw.pack()

tk.Button(root, text="EMERGENCY STOP", bg="red", fg="white", command=stop_motors).pack(pady=10)

# Start loop
root.after(20, send_data)
root.mainloop()
