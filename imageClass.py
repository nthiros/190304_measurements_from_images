class Image(object):
	def __init__(self, img, imggcp=[], realgcp=[],pose=None):
		self.image = img
		self.imagegcp = imggcp
		self.realgcp = realgcp
		self.pose = pose

	