#Team3238 Cyborg Ferrets 2014 Object Detection Code
#Start with
#python image_processor.py 'path/to/image.jpg'
#don't pass an image argument to use the VideoCapture(0) stream.
# Video capture mode updates the frame to process every video_pause milliseconds, so adjust that.
#set enable_dashboard = True to send range and bearing over smart dashboard network_tables interface.
#set show_windows     = False for on-robot, no monitor processing on pandaboard.

#This code is a merge of vision_lib.py, bearing_formula.py, distance_formula.py and team341 java vision detection code from (2012?) competition.

#java -jar SmartDashboard ip 127.0.0.1, for example, will start the dashboard if running on this same host.
#Now tuned for green leds.
#expected camera settings (sorry no numbers on camera interface.)
# exposure -> far right
# gain -> far left
# brightness ~ 20% from left
# contrast ~ 20% from left
# color intensity ~ 18% from left

enable_dashboard = False
show_windows     = True

from cv2 import *
import numpy as np
import sys
import math

if enable_dashboard:
  from pynetworktables import *

if enable_dashboard:
  SmartDashboard.init()
#pretend the robot is on the network reporting its heading to the SmartDashboard,
#  then let the SmartDashboard user modify it and send it back to this code to simulate movement.
robot_heading_title = 'Robot Heading (Deg):'
if enable_dashboard:
  SmartDashboard.PutNumber(robot_heading_title, 0.0)

class ImageProcessor:
  #all these values could be put into the SmartDashboard for live tuning as conditions change.

  default_shape   = (480,640,3)
  h               = np.zeros(default_shape, dtype=np.uint8)
  s               = np.zeros(default_shape, dtype=np.uint8)
  v               = np.zeros(default_shape, dtype=np.uint8)
  combined        = np.zeros(default_shape, dtype=np.uint8)
  img             = np.zeros(default_shape, dtype=np.uint8)
  h_title         = "hue"                 
  s_title         = "sat"                 
  v_title         = "val"                 
  combined_title  = "Combined + Morphed" 
  targets_title   = "Targets" 

  #tuned for the camera settings above and the green leds. (Red didn't work as well and requires changing the threshold function to use OR of inverse and normal threshold, because red is on the top and bottom of the hue scale (wraps around.).)
  hue_delta                   = 15
  sat_delta                   = 25
  val_delta                   = 100
  hue_thresh      = 80
  sat_thresh      = 233
  val_thresh      = 212
  hue_low_thresh  = 020 #center on 80, delta as previous
  hue_high_thresh = 220
  sat_low_thresh  = 160 #center on 233, delta as previous
  sat_high_thresh = 255
  val_low_thresh  = 053 # center on 212, delta as previous
  val_high_thresh = 255
  max_thresh = 255

  #used for the morphologyEx method that fills in the pixels in the combined image prior to identifying polygons and contours.
  kernel     = getStructuringElement(MORPH_RECT, (2,2), anchor=(1,1)) 
  morph_close_iterations = 9

  #colors in BGR format for drawing the targets over the image.
  selected_target_color    = (0,0,255)
  passed_up_target_color   = (255,0,0)
  possible_target_color    = (0,255,0)

  #used to judge whether a polygon side is near vertical or near horizontal, for filtering out shapes that don't match expected target characteristics
  vert_threshold           = math.tan(math.radians(90-20)) 
  horiz_threshold          = math.tan(math.radians(20)) 

  #used to look for only horizontal or vertical rectangles of an aspect ratio that matches the targets.
  #currently open wide to find both horizontal and vertical targets
  max_target_aspect_ratio  = 10 # 1.0 # top target is expected to be 24.5 in x 4 in.
  min_target_aspect_ratio  = 0.1 #0.01# 3# 0.5

  robot_heading               = 0.0 #input from SmartDashboard if enabled, else hard coded here.
  x_resolution                = 640 #needs to match the camera.
  y_resolution                = 480 
  theta                       = math.radians(49.165) # * math.pi/(180 *2.0) #radians, half of field of vision.
  real_target_width           = 2.0 #24 * 0.0254 #1 inch / 0.254 meters target is 24 inches wide
  angle_to_shooter            = 0 

  #not currently using these constants and may not be correct for current robot configuration.
  # target_min_width       = 20
  # target_max_width       = 200
  # degrees_horiz_field_of_view = 47.0                                  
  # degrees_vert_field_of_view  = 480.0/640*degrees_horiz_field_of_view 
  # inches_camera_height        = 54.0                                  
  # inches_top_target_height    = 98 + 2 + 98                           
  # degrees_camera_pitch        = 21.0                                  
  # degrees_sighting_offset     = -1.55                                 
  
  def __init__(self, img_path):
    self.img_path = img_path
    self.layout_result_windows(self.h,self.s,self.v)
    self.vc = VideoCapture(0)


  def video_feed(self):
    while True:
      if self.img is not None:
        self.process()

      if self.img_path is None:
        rval, self.img      = self.vc.read() #might set to None
      else:
        self.img            = imread(self.img_path)


  def process(self):

    if enable_dashboard:
      self.robot_heading = SmartDashboard.GetNumber(robot_heading_title)

    drawing             = np.zeros(self.img.shape, dtype=np.uint8)

    self.hsv               = cvtColor(self.img, cv.CV_BGR2HSV)
    self.h, self.s, self.v = split(self.hsv)
    self.h_clipped         = self.threshold_in_range(self.h, self.hue_thresh-self.hue_delta, self.hue_thresh+self.hue_delta)
    self.s_clipped         = self.threshold_in_range(self.s, self.sat_thresh-self.sat_delta, self.sat_thresh+self.sat_delta)
    self.v_clipped         = self.threshold_in_range(self.v, self.val_thresh-self.val_delta, self.val_thresh+self.val_delta)
    if show_windows:
      imshow(self.h_title, self.h_clipped)
      imshow(self.s_title, self.s_clipped)
      imshow(self.v_title, self.v_clipped)
    self.find_targets()
   
    if waitKey(1000) == ord('q'):
      exit(1)

  def layout_result_windows(self, h, s, v):
    if show_windows:
      pos_x, pos_y        = 500,500               
      # imshow(self.img_path, self.img)
      imshow(self.h_title, h)
      imshow(self.s_title, s)
      imshow(self.v_title, v)
      imshow(self.combined_title, self.combined)
      imshow(self.targets_title, self.img)

      moveWindow(self.h_title, pos_x*1, pos_y*0);
      moveWindow(self.s_title, pos_x*0, pos_y*1);
      moveWindow(self.v_title, pos_x*1, pos_y*1);
      moveWindow(self.combined_title, pos_x*2, pos_y*0);
      # moveWindow(self.targets_title, pos_x*2, pos_y*1);

      #these seem to be placed alphabetically....
      # createTrackbar( "Hue High Threshold:", self.source_title, self.hue_high_thresh, self.max_thresh, self.update_hue_high_threshold);
      # createTrackbar( "Hue Low Threshold:", self.source_title, self.hue_low_thresh, self.max_thresh, self.update_hue_low_threshold);
      # createTrackbar( "Sat High Threshold:", self.source_title, self.sat_high_thresh, self.max_thresh, self.update_sat_high_threshold);
      # createTrackbar( "Sat Low Threshold:", self.source_title, self.sat_low_thresh, self.max_thresh, self.update_sat_low_threshold);
      # createTrackbar( "Val High Threshold:", self.source_title, self.val_high_thresh, self.max_thresh, self.update_val_high_threshold);
      # createTrackbar( "Val Low Threshold:", self.source_title, self.val_low_thresh, self.max_thresh, self.update_val_low_threshold);


  def update_hue_threshold(self, thresh):
    delta = 15
    self.h_clipped = self.threshold_in_range(self.h, thresh-delta, thresh+delta)
    imshow(self.h_title, self.h_clipped)
    self.find_targets()

  def update_sat_threshold(self, thresh):
    delta = 25 
    self.s_clipped = self.threshold_in_range(self.s, thresh-delta, thresh+delta)
    imshow(self.s_title, self.s_clipped)
    self.find_targets()

  def update_val_threshold(self, thresh):
    delta = 100
    self.v_clipped = self.threshold_in_range(self.v, thresh-delta, thresh+delta)
    imshow(self.v_title, self.v_clipped)
    self.find_targets()

  def threshold_in_range(self, img, low, high):
    unused, above = threshold(img, low, self.max_thresh, THRESH_BINARY)
    unused, below = threshold(img, high, self.max_thresh, THRESH_BINARY_INV)
    return bitwise_and(above, below)

  def find_targets(self):
    #combine all the masks together to get their overlapping regions.
    if True: 
      self.combined = bitwise_and(self.h_clipped, bitwise_and(self.s_clipped, self.v_clipped))

      #comment above line and uncomment next line to ignore hue channel til we sort out red light hue matching around zero.  
      #self.combined = bitwise_and(self.s_clipped, self.v_clipped)
      
      self.combined = morphologyEx(src=self.combined, op=MORPH_CLOSE, kernel=self.kernel, iterations=self.morph_close_iterations)   
      if show_windows:
        imshow(self.combined_title, self.combined )

      self.contoured      = self.combined.copy() 
      contours, heirarchy = findContours(self.contoured, RETR_LIST, CHAIN_APPROX_TC89_KCOS)
      contours = [convexHull(c.astype(np.float32),clockwise=True,returnPoints=True) for c in contours]
      
      polygon_tuples = self.contours_to_polygon_tuples(contours)        
      polygons       = [self.unpack_polygon(t) for t in polygon_tuples] 

      self.reset_targeting()

      for polygon_tuple in polygon_tuples:
        self.mark_correct_shape_and_orientation(polygon_tuple) 

      if self.selected_target is not None:
        self.draw_target(self.highest_found_so_far_x, self.highest_found_so_far, self.selected_target_color)
        drawContours(self.drawing, [self.unpack_polygon(self.selected_target).astype(np.int32)], -1, self.selected_target_color, thickness=10)
        self.aim()

      if show_windows:
        imshow(self.targets_title, self.drawing)

      if enable_dashboard:
        SmartDashboard.PutNumber("Potential Targets:", len(polygons))


  def aim(self):
    if enable_dashboard:
      self.robot_heading    = SmartDashboard.GetNumber(robot_heading_title)

    polygon, x, y, w, h   = self.selected_target
    self.target_bearing   = self.get_bearing(w)   
    self.target_range     = self.get_range(x, y, w, h)     
    self.target_elevation = self.get_elevation(x, y, w, h) 
    print("Range = " + str(self.target_range))
    print("Bearing = " + str(self.target_bearing))
    if enable_dashboard:
      SmartDashboard.PutNumber("Target Range:",    self.target_range)
      SmartDashboard.PutNumber("Target Bearing:",  self.target_bearing)
      SmartDashboard.PutNumber("Target Elevation:",self.target_elevation)
      SmartDashboard.PutString("Target: ","Acquired!")


  def get_bearing(self, w):
    return self.bearing(w/2.0)*(360/2.0*math.pi)+180

  def get_range(self, x, y, w, h):
    if enable_dashboard:
      SmartDashboard.PutNumber("TargetWidth: ",w)
      SmartDashboard.PutNumber("TargetHeight",h)
      SmartDashboard.PutNumber("TargetX",x)
      SmartDashboard.PutNumber("TargetY",y)

    return self.distance(w)

  def distance(self, pix_width):
    fovr = self.x_resolution * self.real_target_width / pix_width
    if enable_dashboard:
      SmartDashboard.PutNumber("FieldOfViewReal", fovr) # = 2w_real
      SmartDashboard.PutNumber("TanTheta", math.tan(self.theta))
      SmartDashboard.PutNumber("fovr/tan(theta)", fovr/math.tan(self.theta))

    return self.real_target_width*self.x_resolution/(2*pix_width*math.tan(self.theta))

  def bearing(self,x_target):
    return (self.theta*2/self.x_resolution)*(x_target-(self.x_resolution/2))-self.angle_to_shooter


  def get_elevation(self, x, y, w, h):
    return 0.0

  def reset_targeting(self):
    if enable_dashboard:
      SmartDashboard.PutString("Target: ","lost...")
      
    self.drawing                = self.img.copy() 
    self.selected_target        = None            
    self.highest_found_so_far_x = None            
    self.highest_found_so_far   = sys.maxint      
    self.target_range           = 0               
    self.target_bearing         = -1               
    self.target_elevation       = 0               

  def mark_correct_shape_and_orientation(self, polygon_tuple):
    p,x,y,w,h                               = polygon_tuple
    if isContourConvex(p) and 4==len(p) and self.slope_angles_correct(p):
      center_x = int(x + w/2.0)
      center_y = int(y + h/2.0)
      self.draw_target(center_x, center_y, self.possible_target_color)

      if center_y < self.highest_found_so_far:
        self.selected_target = polygon_tuple
        self.highest_found_so_far   = center_y
        self.highest_found_so_far_x = center_x

    else:
      drawContours(self.drawing, [p.astype(np.int32)], -1, self.passed_up_target_color, thickness=7)

  def draw_target(self, center_x, center_y, a_color):
    #circle(self.drawing,(center_x, center_y), radius=10, color=self.selected_target_color, thickness=5)
    radius      = 10 
    a_thickness = 5  
    line(self.drawing, (center_x - radius, center_y), (center_x + radius, center_y), color=a_color, thickness=a_thickness)
    line(self.drawing, (center_x, center_y-radius), (center_x, center_y+radius), color=a_color, thickness=a_thickness)

  def slope_angles_correct(self, polygon):
    num_near_vert, num_near_horiz = 0,0
    for line_starting_point_index in xrange(0,4):
      slope = self.get_slope(polygon, line_starting_point_index)
      if slope < self.horiz_threshold:
        num_near_horiz += 1 
      if slope > self.vert_threshold:
        num_near_vert += 1 

    return 1 <= num_near_horiz and 2 == num_near_vert

  def get_slope(self, p, line_starting_point_index):
    line_ending_point_index = (line_starting_point_index+1)%4
    dy = p[line_starting_point_index, 0, 1] - p[line_ending_point_index, 0, 1]
    dx = p[line_starting_point_index, 0, 0] - p[line_ending_point_index, 0, 0]
    slope = sys.float_info.max
    if 0 != dx:
      slope = abs(float(dy)/dx)

    return slope

  def unpack_polygon(self,t):
    p,x,y,w,h = t
    return p

  def contours_to_polygon_tuples(self, contours):
    polygon_tuples = []
    for c in contours:
      x, y, w, h = boundingRect(c)
      if self.aspect_ratio_and_size_correct(w,h):
        p = approxPolyDP(c, 20, False)
        polygon_tuples.append((p,x,y,w,h))


    return polygon_tuples 

  def aspect_ratio_and_size_correct(self, width, height):
    ratio = float(width)/height #float(height)/width
    return ratio < self.max_target_aspect_ratio and ratio > self.min_target_aspect_ratio #and width > self.target_min_width and width < self.target_max_width
    #note: we don't want to ignore potential targets based on pixel width and height since range will change the pixel coverage of a real target.

 
if '__main__'==__name__:
  try:
    img_path = sys.argv[1]
  except:
    img_path= None
    # print('Please add an image path argument and try again.')
    # sys.exit(2)

  ImageProcessor(img_path).video_feed()
