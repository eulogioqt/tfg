"""TODO: Add module documentation."""
import cv2
import base64

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

from rosidl_runtime_py.convert import message_to_ordereddict


class R2WBridge():

"""TODO: Describe class."""
    def __init__(self):
    """TODO: Describe __init__.
"""
        self.cv_bridge = CvBridge()
    
    def img_to_r2w(self, img):
    """TODO: Describe img_to_r2w.
Args:
    img (:obj:`Any`): TODO.
"""
        img_cv2 = self.cv_bridge.imgmsg_to_cv2(img)
        #encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50] # pasar los bounding box como texto
        ret, jpeg = cv2.imencode('.jpg', img_cv2)#, encode_param)

        if ret:
            return base64.b64encode(jpeg.tobytes()).decode('utf-8')
        else:
            raise Exception("Error on protocol.image_message (not ret)")
    
    def str_to_r2w(self, data):
    """TODO: Describe str_to_r2w.
Args:
    data (:obj:`Any`): TODO.
"""
        return data.data

    def any_to_r2w(self, data):
    """TODO: Describe any_to_r2w.
Args:
    data (:obj:`Any`): TODO.
"""
        if isinstance(data, Image): # esto igual
            return self.img_to_r2w(data)
        if isinstance(data, String): # esto que sea opcional
            return self.str_to_r2w(data)
        else:
            return message_to_ordereddict(data)
