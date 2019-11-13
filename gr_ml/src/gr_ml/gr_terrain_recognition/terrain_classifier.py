from gr_ml.gr_terrain_recognition.coder import   Features2Image
import rospy

class TerrainClassifier(Features2Image):
    def __init__(self):
        Features2Image.__init__(self,ros=True, pix_per_meter=1, msg_selection=3,meters=20, memory=25)#meters=10,pix_per_meter=1, msg_selection=2)
