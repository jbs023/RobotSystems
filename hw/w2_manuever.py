import sys
sys.path.append(r'../lib')
from utils import reset_mcu
reset_mcu()

from picarx import Picarx
import time


def k_turn(px, speed):
    px.set_dir_servo_angle(-30)
    px.forward(speed)
    time.sleep(2)

    px.set_dir_servo_angle(30)
    px.backward(speed)
    time.sleep(1.5)

    px.set_dir_servo_angle(0)
    px.forward(speed)
    time.sleep(1)
    px.forward(0)

def parallel_park(px, speed):
    px.backward(speed)
    time.sleep(0.1)
    for angle in range(0, 15, 1):
        px.set_dir_servo_angle(angle)
        time.sleep(0.01)
    time.sleep(1.75)

    px.set_dir_servo_angle(35)
    px.forward(speed)
    time.sleep(1)
    for angle in range(35, 0, -1):
        px.set_dir_servo_angle(angle)
        time.sleep(0.0075)

    time.sleep(0.3)
    px.backward(speed)
    time.sleep(0.5)
    px.backward(0)

def forward_and_backwards(px, speed, sleep_time):
    px.forward(speed)
    time.sleep(sleep_time)
    px.backward(speed)
    time.sleep(sleep_time)
    px.backward(0)

if __name__ == "__main__":
    px = Picarx()
    while True:
        print("Please select a manuever:")
        print("\t1: Forwards and Backwards")
        print("\t2: Parallel Park")
        print("\t3: K-Turn")
        print("\tq: Quit program")
        print()
        option = input("Enter an manuever: ")

        if option == "1":
            forward_and_backwards(px, 50, 3)
        elif option == "2":
            parallel_park(px, 50)
        elif option == "3":
            k_turn(px, 50)
        elif option == "q":
            exit()
        else:
            print()
            print("Error: {} is invalid Option.".format(option))
        



