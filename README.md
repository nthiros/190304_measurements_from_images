# 190304_measurements_from_images

	HEIRARCHY:
		
		CLASS PHOTOGRAM():
		
			def Estimate_realworldcoords():
			def Residual_realworldcoords():

			CLASS CAMERA():
				
				self.focallength
				self.camerapose
				self.sensor
				self.images *list*

				def Rotational_transform():
				def Projective_transform():
				def Residual():
				def EstimatePose():

				CLASS IMAGE():
					
					self.image
					self.groundcontrolpoints
					self.imagecoords
