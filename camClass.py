class Camera_pose(object):
    def __init__(self):
        self.p = None                   # Pose
        self.f = None                   # Focal Length in Pixels
        self.c = np.array([None,None])  # Sensor
        
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
          
    def residual(self, pose, pts, u_gcp):
        ''' Calculate the residual between transformed coordinates on prominant features and measured pixel values of those features'''
        pt = myCam.projective_transform(myCam.rotational_transform(pts,pose))
        res = pt.flatten() - u_gcp.flatten()
        return res    
    def estimate_pose(self, pts, u_gcp):
        """
        This function adjusts the pose vector such that the difference between the observed pixel coordinates u_gcp 
        and the projected pixels coordinates of X_gcp is minimized.
        """
        p_opt = so.least_squares(self.residual, self.p, method='lm',args=(pts,u_gcp))
        return p_opt