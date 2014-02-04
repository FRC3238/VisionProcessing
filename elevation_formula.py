class ImageProcessor:
    degrees_vert_field_of_view = 44.26 
    y_resolution = 480.0
    angle_to_shooter = 0.00
    def bearing(self,y_target):
        return (self.degrees_vert_field_of_view/self.y_resolution)*(y_target-(self.y_resolution/2.0))-self.angle_to_shooter
        
