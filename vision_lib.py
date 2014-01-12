from cv2 import *
from urllib2 import urlopen
import numpy as np
from cStringIO import StringIO


def get_image_from_axis():
    url = "http://10.32.38.11/axis-cgi/jpg/image.cgi"
    request = urlopen(url)
    img_array = np.asarray(bytearray(request.read()), dtype=np.uint8)
    return imdecode(img_array, CV_LOAD_IMAGE_COLOR)
