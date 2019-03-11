import numpy as np
import matplotlib.pyplot as plt
import PIL.Image
import matplotlib.image as mpimg
import scipy.optimize as so


class Image(object):
    def __init__(self, img, imggcp=[], realgcp=[],pose=None):
        self.image = img
        self.imagegcp = imggcp
        self.realgcp = realgcp
        self.pose = pose

class camClass(Image):
    def __init__(self, foc_len, sensor_x, sensor_y, pose_guess):
        self.f = foc_len                           # Focal Length in Pixels
        self.c = np.array([sensor_x,sensor_y])  # Sensor
        self.images = []
        self.pose_guess = pose_guess

    def add_images(self,image):
        image.pose = self.pose_guess  #initialize image with guess
        self.images.append(image) 

    def rotational_transform(self,pts,pose):
            """  
            This function performs the translation and rotation from world coordinates into generalized camera coordinates.
            This function takes the Easting, Northing, and Elevation of the features in an image.
            The pose vector is unknown and what we are looking to optimize.
            """
            cam_x = pose[0]
            cam_y = pose[1]
            cam_z = pose[2]
            roll = pose[3]
            pitch = pose[4]
            yaw = pose[5]

            r_axis = np.array([[1, 0, 0], 
                               [0, 0,-1], 
                               [0, 1, 0]])
            r_roll = np.array([[np.cos(roll), 0, -1*np.sin(roll)], 
                               [0, 1, 0], 
                               [np.sin(roll), 0, np.cos(roll)]])            
            r_pitch = np.array([[1, 0, 0], 
                                [0, np.cos(pitch), np.sin(pitch)], 
                                [0, -1*np.sin(pitch), np.cos(pitch)]])           
            r_yaw = np.array([[np.cos(yaw), -1*np.sin(yaw), 0, 0], 
                              [np.sin(yaw), np.cos(yaw), 0, 0], 
                              [0, 0, 1, 0]])
            T = np.array([[1, 0, 0, -cam_x], 
                          [0, 1, 0, -cam_y], 
                          [0, 0, 1, -cam_z], 
                          [0, 0, 0, 1]])
            C = r_axis @ r_roll @ r_pitch @ r_yaw @ T   
            
            
            if pts.ndim <= 1:
                pts = pts[np.newaxis,:]
            pts = (np.c_[pts, np.ones(pts.shape[0])]).T
            
            return C @ pts
               
    def projective_transform(self,rot_pt):
        """  
        This function performs the projective transform on generalized coordinates in the camera reference frame.
        This function needs the outputs of the rotational transform function (the rotated points).
        """
        focal = self.f 
        sensor = self.c  
        rot_pt = rot_pt.T
        #General Coordinates
        gcx = rot_pt[:,0]/rot_pt[:,2]
        gcy = rot_pt[:,1]/rot_pt[:,2]
        #Pixel Locations
        pu = gcx*focal + sensor[0]/2.
        pv = gcy*focal + sensor[1]/2.
        return np.array([pu,pv]).T
          
 
    def estimate_pose(self):
        
         def residual_pose(pose, realgcp, imagegcp,self):
            pt = self.projective_transform(self.rotational_transform(realgcp, pose))
            res = pt.flatten() - imagegcp.flatten()
            return res 

         for i in range(len(self.images)):
            realgcp = self.images[i].realgcp
            imagegcp = self.images[i].imagegcp
            self.images[i].pose = so.least_squares(residual_pose, self.images[i].pose, method='lm',args=[realgcp, imagegcp,self]).x
     
    def estimate_RWC(self):

        if(len(self.images) < 2):
            print("There are not 2 images in this camera class")
        #===========================
        def residual_RWC( RWC, pose1, pose2,imcor1,imcor2,self):
        
            pt_1 = self.projective_transform(self.rotational_transform(RWC, pose1)) # u,v based on first image
            pt_2 = self.projective_transform(self.rotational_transform(RWC, pose2)) 
            res_1 = pt_1.flatten() - imcor1.flatten()
            res_2 = pt_2.flatten() - imcor2.flatten()
            return np.hstack((res_1, res_2))  
        #==========================
    
        for i in range(len(self.images)):
            for j in range(len(self.images)):
                if( i != j):
                    self.images[i].realgcp = so.least_squares(residual_RWC, self.images[i].realgcp , method='lm',args=(self.images[i].pose, self.images[j].pose, self.images[i].imagegcp, self.images[j].imagegcp,self)).x        
 
