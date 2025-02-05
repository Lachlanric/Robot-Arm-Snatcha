import tkinter as tk
import serial
from math import sqrt, atan2, acos, pi, radians, degrees
from time import sleep

# Create the main application window
root = tk.Tk()
root.title("Tkinter Slider Example")

ser = serial.Serial(
    port='COM5',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0.1
)

motorAngleLerps = [
    [(90,74), (180,219)],
    [(0,20),  (90,184)],
    [(0,25),  (180,246)],
    [(0,92),  (90,215)],
    [(0,255),  (255,0)]
]

def angleToMotorVal(ang, motorIdx):

    x1,y1 = motorAngleLerps[motorIdx][0]
    x2,y2 = motorAngleLerps[motorIdx][1]
    m = (y2-y1)/(x2-x1)

    motorVal = y1 + m*(ang - x1)
    return motorVal


# y = height

def print_vals(dummy):
    motorVals = None
    try:
        motorVals = calculate_motor_vals()
        ser.write(bytes(motorVals))
        # for i,value in enumerate(motorVals):
        #     ser.write(bytes([i, int(value)]))


    except ValueError as e:
        print(e)
    finally:
        print(motorVals)
    

def calculate_motor_vals():

    X,Y,Z,W,C = slider_X.get(), slider_Y.get(), slider_Z.get(), radians(slider_W.get()), radians(slider_C.get())
    print(f"{X}, {Y}, {Z}, {W}")

    L = 10
    h = 10

    R = sqrt(X**2 + Z**2)
    print(f"R =     {R}")
    ang = atan2(Z,X)
    print(f"ang =   {ang}")

    q = sqrt(R**2 + (Y-h)**2)
    print(f"q =     {q}")
    alpha = atan2(Y-h,R)
    print(f"alpha = {alpha}")
    theta = acos(q/(2*L)) + alpha
    print(f"theta = {theta}")

    phi = pi - 2*(theta-alpha)
    print(f"phi =   {phi}")
    psi = -(theta-pi+phi) + W
    print(f"psi =   {psi}")
    print("\n")

    motorAngles = [phi, theta, ang, psi, C]  # in radians
    motorVals = [int(angleToMotorVal(degrees(val),i)) for i,val in enumerate(motorAngles)]

    print(motorAngles)
    print([degrees(m) for m in motorAngles])
    
    # Clamp wrist angle
    motorVals[3] = min(255,max(0,motorVals[3]))
    
    for val in motorVals:
        if val<0 or val>255:
            raise ValueError("Motor values out of range:",motorVals)
    
    return motorVals




l = tk.Label(root, text = "X")
l.pack()
slider_X = tk.Scale(
    root,
    from_=-20,          # Minimum value of the slider
    to=20,           # Maximum value of the slider
    orient="horizontal",  # Orientation of the slider
    length=500,       # Length of the slider in pixels
    command=print_vals
)
slider_X.pack(pady=10)
slider_X.set(10)

l = tk.Label(root, text = "Y")
l.pack()
slider_Y = tk.Scale(
    root,
    from_=0,          # Minimum value of the slider
    to=30,           # Maximum value of the slider
    orient="horizontal",  # Orientation of the slider
    length=500,       # Length of the slider in pixels
    command=print_vals
)
slider_Y.pack(pady=10)
slider_Y.set(10)

l = tk.Label(root, text = "Z")
l.pack()
slider_Z = tk.Scale(
    root,
    from_=0,          # Minimum value of the slider
    to=20,           # Maximum value of the slider
    orient="horizontal",  # Orientation of the slider
    length=500,       # Length of the slider in pixels
    command=print_vals
)
slider_Z.pack(pady=10)
slider_Z.set(10)

l = tk.Label(root, text = "Wrist Angle")
l.pack()
slider_W = tk.Scale(
    root,
    from_=-90,          # Minimum value of the slider
    to=90,           # Maximum value of the slider
    orient="horizontal",  # Orientation of the slider
    length=500,       # Length of the slider in pixels
    command=print_vals
)
slider_W.pack(pady=10)
slider_W.set(0)

l = tk.Label(root, text = "Claw")
l.pack()
slider_C = tk.Scale(
    root,
    from_=0,          # Minimum value of the slider
    to=255,           # Maximum value of the slider
    orient="horizontal",  # Orientation of the slider
    length=500,       # Length of the slider in pixels
    command=print_vals
)
slider_C.pack(pady=10)
slider_C.set(128)



# Start the server
if __name__ == '__main__':

    try:
        # Run the application
        root.mainloop()
    except KeyboardInterrupt:
        pass
    # finally:
    #     ser.close()

