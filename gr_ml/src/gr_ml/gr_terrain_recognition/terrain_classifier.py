from gr_ml.gr_terrain_recognition.pc_to_image import   Lidar2Image
import rospy

class TerrainClassifier(Lidar2Image):
    def __init__(self):
        Lidar2Image.__init__(self,ros=True, meters=10, pix_per_meter=1)

