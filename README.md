# StereoShape
C++ code for 3D shape recontruction from stereo camera.  

This plugin can be used to generate 3D shape from a pair of images. 
Calibration matrices are needed.
You can build a 3D triangulated mesh using Moving Least Square. 
This will filter out some outliers and result in an OBJ file.
This work was initially designed for laparosopic images.


![3D shape recovery from stereo laparoscopic images](https://github.com/rouge1616/StereoShape/blob/master/StereoShape.png)
