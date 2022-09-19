import csv
from matplotlib import pyplot as plt
sampling_time=0.01
e_previous=0
I_previous=0
D_previous=0
v=0
v_list=[]
out_list=[]

with open("racetrack_waypoints.txt") as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')

    for line in csv_reader:
        v_list.append(line[2])

kp=int(input("enter kp : "))
ki=int(input("enter ki : "))
kd=int(input("enter kd : "))

for v_desired in v_list:
    e=float(v_desired)-v
    P=kp * e
    I=((ki*sampling_time/2)*(e + e_previous)) + I_previous
    D=((2*kd/sampling_time)*(e - e_previous)) + D_previous

    v=(P+I+D)*sampling_time + v

    out_list.append(v)
    D_previous = D
    I_previous = I
    e_previous = e
    print(e)

'''
plt.figure()
plt.plot(v_list)
plt.plot(out_list)
plt.legend()
plt.show()
'''
