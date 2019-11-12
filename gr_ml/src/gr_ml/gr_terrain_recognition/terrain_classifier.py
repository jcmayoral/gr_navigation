from gr_ml.gr_terrain_recognition.coder import   Features2Image
import rospy

class TerrainClassifier(Features2Image):
    def __init__(self):
        Features2Image.__init__(self,ros=True, meters=10, pix_per_meter=1)

