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

    #    vc = VideoCapture(0)

    ## load pic
    #if vc.isOpened(): # try to get the first frame
    #  rval, frame = vc.read()
    #else:
    #  rval = False

    
    #rval, self.img

    if self.firstRun is True:
      self.videoCam = VideoCapture(0)
      firstRun = False


    while True:
        if self.videoCam.isOpened():
            rval, img = self.videoCam.read()
            break

    rval, img = self.videoCam.read()

    return img
