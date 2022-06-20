import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import csv
import numpy as np
import time

# Create figure for plotting
fig, axs = plt.subplots(3, 2)
xs = []
ys1 = []
ys2 = []
ys3 = []
ys4 = []
ys5 = []
ys6 = []

cnt = 0

gravity = 9.78197

# This function is called periodically from FuncAnimation
def animate(i, xs, ys1, ys2, ys3, ys4, ys5, ys6):
    global cnt
    # Read temperature (Celsius) from TMP102
    with open("imuData.csv", 'r') as file:
        csvreader = csv.reader(file)
        header = next(csvreader)
        print(header)
        for row in csvreader:
            rows = row 
        cnt += 1

    # Add x and y to lists
    xs.append(dt.datetime.now().strftime('%H:%M:%S%f'[:-2]))
    ys1.append(int(rows[0])) #/16384*gravity)
    ys2.append(int(rows[1])) #/16384*gravity)
    ys3.append(int(rows[2])) #/16384*gravity)
    ys4.append(int(rows[3])) #/16.4)
    ys5.append(int(rows[4])) #/16.4)
    ys6.append(int(rows[5])) #/16.4)
    print(rows)

    if cnt > 10 :
        ys1.pop(0)
        ys2.pop(0)
        ys3.pop(0)
        ys4.pop(0)
        ys5.pop(0)
        ys6.pop(0)
        xs.pop(0)
        cnt = 6

    # Draw x and y lists
    axs[0, 0].clear()
    axs[1, 0].clear()
    axs[2, 0].clear()
    axs[0, 1].clear()
    axs[1, 1].clear()
    axs[2, 1].clear()
        
    axs[0, 0].plot(xs, ys1,color='purple')
    axs[0, 0].set_title("accel_x")

    axs[1, 0].plot(xs, ys2,color='blue')
    axs[1, 0].set_title("accel_y")

    axs[2, 0].plot(xs, ys3,color='green')
    axs[2, 0].set_title("accel_z")

    axs[0, 1].plot(xs, ys4 ,color='yellow')
    axs[0, 1].set_title("anglvel_x")

    axs[1, 1].plot(xs, ys5,color='orange')
    axs[1, 1].set_title("anglvel_y")

    axs[2, 1].plot(xs, ys6,color='red')
    axs[2, 1].set_title("anglvel_z")


    list = ['x-accel (m/s^2)', 'x-gyro (radians/s^2)', 'y-accel (m/s^2)',  'y-gyro (radians/s^2)','z-accel (m/s^2)', 'z-gyro (radians/s^2)']
    i = 0
    for ax in axs.flat:
        ax.set(xlabel='time (Hour:Minute:Second)', ylabel=str(list[i]))
        i+=1
    
        


ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys1, ys2, ys3, ys4, ys5, ys6), interval=500, repeat=True)
plt.tight_layout()
plt.show()

