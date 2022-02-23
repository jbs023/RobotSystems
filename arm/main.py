import cv2
import logging
import argparse
import threading

from move import Move
from camera import Camera
from perception import Perception
from shared_state import SharedState

def main(config):
    #Line following simultaneity
    if config.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    #Init arm and camera objects
    target_colors = ('red', 'green', 'blue')
    shared_state = SharedState(target_colors, config.function)
    shared_state.start()

    perception = Perception(shared_state)
    move = Move(shared_state)
    camera = Camera()
    camera.camera_open()

    #Set up functions to use
    percept_func = perception.identify_single_color
    if config.function != "1":
        percept_func = perception.identify_multiple_colors

    move_func = move.move_block
    if config.function == "2":
        logging.debug("Sort Blocks")
        move_func = move.sort_blocks
    elif config.function == "3":
        logging.debug("Stack Blocks")
        move_func = move.stack_blocks
    elif config.function != "1":
        print("Invalid Function Choice!")
        exit()

    # Use the threads the same way original code did
    # they share too much information to quickly integrate
    # a consumer-producer framework

    #Start move thread
    move_thread = threading.Thread(target=move_func, daemon=True)
    move_thread.start()

    #Start camera thread
    camera_threa = threading.Thread(target=camera.camera_task, args=(), daemon=True)
    camera_threa.start()

    #Start main thread, which executes the run function
    while True:
        img = camera.frame
        if img is not None:
            frame = img.copy()
            Frame = percept_func(frame)           
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    camera.camera_close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--debug', action='store_true',
                        help='Debug flag')
    parser.add_argument('-f', '--function', default=1,  help='1. Move block \n2. Sort Blocks \n3. Palletize Blocks')
    main(parser.parse_args())