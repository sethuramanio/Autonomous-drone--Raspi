import time
from SunFounder_PCA9685 import Servo
myservo= []
c1=11;
c2=8;
c3=7; #base is 50
c4=4;
c5=3;
c6=1;

c1_b=50
c2_b=140
c3_b=140
c4_b=120
c5_b=90
c6_b=100


for i in range(0, 16):
        myservo.append(Servo.Servo(i))  # channel 1
        Servo.Servo(i).setup()
        #print ('myservo%s'%i)

def base_position():
    myservo[c1].write(90)
    myservo[c2].write(90)
    myservo[c3].write(60)#60
    myservo[c4].write(50)#50
    #myservo[c5].write(90)
    #myservo[c6].write(100)
    print("Arm Ready!")

def move_forward():
    print("Moving Forward")
    c3_b=60
    c4_b=50
    for i in range(0, 100):
        c3_b+=1.1
        c4_b+=0.75
        myservo[c3].write(c3_b)
        myservo[c4].write(c4_b)
        time.sleep(0.05)
    time.sleep(0.5)
    myservo[c2].write(180)
    time.sleep(1)
    myservo[c2].write(90)

def move_back():
    print("Move Back")
    c3_b = 170
    c4_b = 125
    for i in range(100, 1, -1):
        c3_b-=1.2
        c4_b-=0.75
        myservo[c3].write(c3_b)
        myservo[c4].write(c4_b)
        time.sleep(0.05)


base_position()

