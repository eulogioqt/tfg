import cv2
import json
import base64

def image_message(image):
    ret, jpeg = cv2.imencode('.jpg', image)

    if ret:
        frame_base64 = base64.b64encode(jpeg.tobytes()).decode('utf-8')

        message = {
            "type": "DISPLAY_DATA",
            "data": {
                "type": "IMAGE",
                "value": frame_base64
            }
        }
    
        return json.dumps(message)
    else:
        raise Exception("Error on protocol.image_message (not ret)")