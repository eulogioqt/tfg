import cv2
import base64

from cv_bridge import CvBridge

class R2WBridge():

    def __init__(self):
        self.cv_bridge = CvBridge()
    
    def img_to_r2w(self, img):
        img_cv2 = self.cv_bridge.imgmsg_to_cv2(img)
        ret, jpeg = cv2.imencode('.jpg', img_cv2)

        if ret:
            return base64.b64encode(jpeg.tobytes()).decode('utf-8')
        else:
            raise Exception("Error on protocol.image_message (not ret)")
        
    def any_to_r2w(self, data):
        if str(type(data)) == "<class 'sensor_msgs.msg._image.Image'>":
            return self.img_to_r2w(data)
        else:
            return data