class ImageProcessor:
    field_of_view = 47.0
    x_resolution = 640
    angle_to_shooter = 0
    def bearing(self,x_target):
        return (self.field_of_view/self.x_resolution)*(x_target-(self.x_resolution/2))-self.angle_to_shooter
        
