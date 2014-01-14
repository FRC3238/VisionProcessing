#Team3238 Cyborg Ferrets Pre-season Vision Object Detection Tutorial
#Standing on the shoulders (or at least the code) of teams 294, 341, and 2423,
#the goal of this tutorial is to help students learn how to tune their vision targeting algorithms.
#Don't forget to start up the SmartDashboard to interact with the algorithm in real time.
#java -jar SmartDashboard ip 127.0.0.1, for example, if running on this same host.
#Some possible assignments:
#  Add more tuning knobs to gui for threshold window delta values.
#  Add signals from SmartDashboard to modify selection algorithm from highest to 
#    left-most, right-most, or lowest targets.
#  Update to use video instead of still imagery to tune for changing lighting and conditions.  
#  Measure response times over network.
#  Measure bandwidth over network.
#  Add SmartDashboard controls to limit thresholding and merging to single channels 
#    and measure timing improvements.
#  Tune camera image acquisition settings on real targets for various lighting conditions.
#  Fill in methods for target bearing, range, and elevation, re-tuned for our live camera and actual targets.
#  Re-write selection rules and color tuning for images from other past FRC competitions.
#  Write robot response code to act on targeting info, connect and see what happens.
#  Write robot code to send its heading to the vision processing code via NetworkTables.

from cv2 import *
import numpy as np
import sys
import math
#from pynetworktables import *

#SmartDashboard.init()
#pretend the robot is on the network reporting its heading to the SmartDashboard,
#  then let the SmartDashboard user modify it and send it back to this code to simulate movement.
robot_heading_title = 'Robot Heading (Deg):'
#SmartDashboard.PutNumber(robot_heading_title, 0.0)

class ImageProcessor:
  #all these values could be put into the SmartDashboard for live tuning as conditions change.
  hue_thresh = 070
  sat_thresh = 180
  val_thresh = 253
  max_thresh = 255
  hsv_calced = False
  kernel     = getStructuringElement(MORPH_RECT, (2,2), anchor=(1,1)) 
  morph_close_iterations = 9
  target_min_width       = 20
  target_max_width       = 200
  max_target_aspect_ratio  = 1.0
  min_target_aspect_ratio  = 0.5
  selected_target_color    = (0,0,255)
  passed_up_target_color   = (255,255,0)
  possible_target_color    = (255,0,255)
  vert_threshold           = math.tan(math.radians(90-20)) 
  horiz_threshold          = math.tan(math.radians(20)) 
  degrees_horiz_field_of_view = 47.0
  degrees_vert_field_of_view  = 480.0/640*degrees_horiz_field_of_view
  inches_camera_height        = 54.0
  inches_top_target_height    = 98 + 2 + 98
  degrees_camera_pitch        = 21.0
  degrees_sighting_offset     = -1.55
  robot_heading               = 0.0


  def __init__(self, img_path):
    self.img_path = img_path

  def process(self):
    #self.robot_heading = SmartDashboard.GetNumber(robot_heading_title)
    self.img            = imread(self.img_path)
    drawing             = np.zeros(self.img.shape, dtype=np.uint8)
    self.source_title   = self.img_path         
    self.h_title        = "hue"                 
    self.s_title        = "sat"                 
    self.v_title        = "val"                 
    self.combined_title = "Combined + Morphed" 
    self.targets_title  = "Targets" 

    hsv = cvtColor(self.img, cv.CV_BGR2HSV)
    self.h, self.s, self.v = split(hsv)
    self.update_hue_threshold(self.hue_thresh)
    self.update_sat_threshold(self.sat_thresh)
    self.update_val_threshold(self.val_thresh)
    self.hsv_calced = True
    self.find_targets()


    self.layout_result_windows(self.h,self.s,self.v)
   
    waitKey(0)

  def layout_result_windows(self, h, s, v):
    pos_x, pos_y        = 500,500               
    imshow(self.img_path, self.img)
    imshow(self.h_title, h)
    imshow(self.s_title, s)
    imshow(self.v_title, v)
    imshow(self.combined_title, self.combined)
    imshow(self.targets_title, self.img)

    moveWindow(self.h_title, pos_x*1, pos_y*0);
    moveWindow(self.s_title, pos_x*0, pos_y*1);
    moveWindow(self.v_title, pos_x*1, pos_y*1);
    moveWindow(self.combined_title, pos_x*2, pos_y*0);
    moveWindow(self.targets_title, pos_x*2, pos_y*1);

    #these seem to be placed alphabetically....
    createTrackbar( "Hue Threshold:", self.source_title, self.hue_thresh, self.max_thresh, self.update_hue_threshold);
    createTrackbar( "Sat Threshold:", self.source_title, self.sat_thresh, self.max_thresh, self.update_sat_threshold);
    createTrackbar( "Val Threshold:", self.source_title, self.val_thresh, self.max_thresh, self.update_val_threshold);

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
    if (self.hsv_calced): 
      self.combined = bitwise_and(self.h_clipped, bitwise_and(self.s_clipped, self.v_clipped))
      self.combined = morphologyEx(src=self.combined, op=MORPH_CLOSE, kernel=self.kernel, iterations=self.morph_close_iterations)   
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

      imshow(self.targets_title, self.drawing)
      #SmartDashboard.PutNumber("Potential Targets:", len(polygons))


  def aim(self):
    self.robot_heading    = SmartDashboard.GetNumber(robot_heading_title)
    polygon, x, y, w, h   = self.selected_target
    self.target_bearing   = self.get_bearing(x, y, w, h)   
    self.target_range     = self.get_range(x, y, w, h)     
    self.target_elevation = self.get_elevation(x, y, w, h) 
    #SmartDashboard.PutNumber("Target Range:",    self.target_range)
    #SmartDashboard.PutNumber("Target Bearing:",  self.target_bearing)
    #SmartDashboard.PutNumber("Target Elevation:",self.target_elevation)
    #SmartDashboard.PutString("Target: ","Acquired!")

  def get_bearing(self, x, y, w, h):
    return 0.0

  def get_range(self, x, y, w, h):
    return 0.0

  def get_elevation(self, x, y, w, h):
    return 0.0

  def reset_targeting(self):
    #SmartDashboard.PutString("Target: ","lost...")
    self.drawing                = self.img.copy() 
    self.selected_target        = None            
    self.highest_found_so_far_x = None            
    self.highest_found_so_far   = sys.maxint      
    self.target_range           = 0               
    self.target_bearing         = 0               
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
    ratio = float(height)/width
    return ratio < self.max_target_aspect_ratio and ratio > self.min_target_aspect_ratio and width > self.target_min_width and width < self.target_max_width

 
if '__main__'==__name__:
  try:
    img_path = "C:\\FRC\\to_furin\\hoops16.jpg" #sys.argv[1]
  except:
    print('Please add an image path argument and try again.')
    sys.exit(2)

  ImageProcessor(img_path).process()
