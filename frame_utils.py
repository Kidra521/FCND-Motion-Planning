import numpy as np 

def get_global_home(filename):
    """
    Get the latitude and longtitude from the collider.csv
    """
    lat, lon = 0, 0

    with open(filename, 'r') as fd:
        for n, line in enumerate(fd):
            if (n == 1):
                lat, lon = [float(x) for x in line.split(",")]
                break

    return lat, lon


