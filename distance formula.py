Python 2.7.6 (default, Nov 10 2013, 19:24:18) [MSC v.1500 32 bit (Intel)] on win32
Type "copyright", "credits" or "license()" for more information.
>>> theta = 23.5
>>> real_target_width = .5
>>> x_resolution = 640
>>> pixel_target_width = 10
>>> distance = real_target_width * x

Traceback (most recent call last):
  File "<pyshell#4>", line 1, in <module>
    distance = real_target_width * x
NameError: name 'x' is not defined
>>> distance = real_target_width * x_resolution/(2* pixel_target_width* math.tan(theta))

Traceback (most recent call last):
  File "<pyshell#5>", line 1, in <module>
    distance = real_target_width * x_resolution/(2* pixel_target_width* math.tan(theta))
NameError: name 'math' is not defined
>>> import math
>>> distance = real_target_width * x_resolution/(2* pixel_target_width* math.tan(theta))
>>> distance
0.9923880764749886
>>> def distance:
	
SyntaxError: invalid syntax
>>> def distance(self):
	return 5

>>> def distance (self,pix_width):
	return real_target_width * x_resolution/(2* pix_width* math.tan(theta))

>>> distance(5)

Traceback (most recent call last):
  File "<pyshell#22>", line 1, in <module>
    distance(5)
TypeError: distance() takes exactly 2 arguments (1 given)
>>> class Container:
	def distance (self,pix_width):
		return real_target_width * x_resolution/(2* pix_width* math.tan(theta))

	
>>> Container.distance(5)

Traceback (most recent call last):
  File "<pyshell#26>", line 1, in <module>
    Container.distance(5)
TypeError: unbound method distance() must be called with Container instance as first argument (got int instance instead)
>>> c = Container()
>>> c.distance(5)
1.9847761529499772
>>> c.distance(100)
0.09923880764749886
>>> class Camera:
	x_resolution = 640
	theta=23.5
	real_target_width=.5
	def distance(self,pix_width):
		return self.real_target_width*self.x_resolution/(2*pix_width*math.tan(self.theta))

	
>>> class Camera:
	x_resolution = 640
	theta=23.5
	real_target_width=.5
	def distance(self,pix_width):
		return self.real_target_width*self.x_resolution/(2*pix_width*math.tan(self.theta))
	def __init__(self,x_resolution,real_target_width,theta)
	
SyntaxError: invalid syntax
>>> class Camera:
	def distance(self,pix_width):
		return self.real_target_width*self.x_resolution/(2*pix_width*math.tan(self.theta))
	def __init__(self,x_resolution,real_target_width,theta):
		self.x_resolution  = x_resolution
		self.theta = theta
		self.real_target_width = real_target_width

		
>>> a=Camera( x_resolution=640, real_target_width=.5,theta=23.5)
>>> a
<__main__.Camera instance at 0x02C8FAF8>
>>> a.x_resolution
640
>>> a.inspect

Traceback (most recent call last):
  File "<pyshell#50>", line 1, in <module>
    a.inspect
AttributeError: Camera instance has no attribute 'inspect'
>>> a.attributes

Traceback (most recent call last):
  File "<pyshell#51>", line 1, in <module>
    a.attributes
AttributeError: Camera instance has no attribute 'attributes'
>>> a.dir()

Traceback (most recent call last):
  File "<pyshell#52>", line 1, in <module>
    a.dir()
AttributeError: Camera instance has no attribute 'dir'
>>> dir(a)
['__doc__', '__init__', '__module__', 'distance', 'real_target_width', 'theta', 'x_resolution']
>>> locals(a)

Traceback (most recent call last):
  File "<pyshell#54>", line 1, in <module>
    locals(a)
TypeError: locals() takes no arguments (1 given)
>>> a.locals()

Traceback (most recent call last):
  File "<pyshell#55>", line 1, in <module>
    a.locals()
AttributeError: Camera instance has no attribute 'locals'
>>> a.__dict__()

Traceback (most recent call last):
  File "<pyshell#56>", line 1, in <module>
    a.__dict__()
TypeError: 'dict' object is not callable
>>> a.__dict__
{'real_target_width': 0.5, 'x_resolution': 640, 'theta': 23.5}
>>> b = Camera(real_target_width=0.5,theta=20,x_resolution=600)
>>> b.__dict__
{'real_target_width': 0.5, 'x_resolution': 600, 'theta': 20}
>>> a.distance(5)
1.9847761529499772
>>> b.distance(5)
13.4098532684675
>>> 
