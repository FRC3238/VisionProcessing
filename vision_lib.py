from cv2 import *
from urllib2 import urlopen
import numpy as np
from cStringIO import StringIO


# Used for getting pictures
class Camera:
  def __init__(self):
    self.firstRun = True
    self.loops = 0

  def from_axis(self):
      
    url = "http://10.32.38.11/axis-cgi/jpg/image.cgi"
    request = urlopen(url)
    img_array = np.asarray(bytearray(request.read()), dtype=np.uint8)
    return imdecode(img_array, CV_LOAD_IMAGE_COLOR)

  def from_file(self, img_path):
    return imread(self.img_path)

  def from_webcam(self):
    if firstRun is True:
      self.videoCam = VideoCapture(0)
      firstRun = False

    rval, img = vc.read()

    return img

#test