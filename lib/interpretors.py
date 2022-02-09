import sys
import logging
import numpy as np

sys.path.append(r'../lib')
from utils import reset_mcu

reset_mcu()

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.ERROR , datefmt="%H:%M:%S ")

class Grayscale_Interpreter():
    """
    Class to interpret the values from the greyscale cameras
    ...

    Attributes
    ----------
    sensitivity : int
        the expected difference between the line and the rest of the floor
    polarity : [1,-1]
        Direction of the sensitivity. Positive will look for a darker 
        line and negative will look for a lighter line
    """

    def __init__(self, sensitivity=0, polarity=-1):
        self.sens = sensitivity
        self.pol = polarity

    def get_rel_dir(self, adc_list):
        """
        Function to process the grayscale data
        :param adc_list: The array of grayscale data from the car
        """
        # Normalize the array to the maximum value obtained
        gry_list_norm = [float(i)/max(adc_list) for i in adc_list]
        gry_list_diff = max(gry_list_norm)-min(gry_list_norm)
        logging.debug("Norm ADC List: {}".format(gry_list_norm))
        logging.debug("Grey Diff: {}".format(gry_list_diff))

        # If the difference is larger than the tolerance, try to detect an edge
        if gry_list_diff > self.sens:
            rel_dir = gry_list_norm[0]-gry_list_norm[2]

            # Calculate the amount of error to make a more continuous relative
            # direction. The deviation of the max or min value from the avg is
            # determined to be the error.
            if self.pol == 1:
                error = (max(gry_list_norm)-np.mean(gry_list_norm))*(2/3)
            elif self.pol == -1:
                error = (min(gry_list_norm)-np.mean(gry_list_norm))*(2/3)
                
            # The relative direction is then multiplied by error and polarity
            # to make a distinction between "just off-centered" and "very off-
            # centered"
            rel_dir_pol = rel_dir*error*self.pol
        else:
            rel_dir_pol = 0

        logging.debug("Rel Dir: {}".format(rel_dir_pol))
        return rel_dir_pol

class UltraSonic_Interpreter():
    def __init__(self, stopping_distance=10):
        #Stopping distance in cm
        self.threshold = stopping_distance
        pass

    def interpret(self, dist):
        '''If distance to object is less than threshold, then stop'''
        if dist > 0 and dist < self.threshold:
            return True
        else:
            return False
