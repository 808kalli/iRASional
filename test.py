import cv2
import base64
import numpy as np



from multiprocessing import Process, Pipe
import time

def process1(pipe):
    request = cv2.imread("image.jpg")
    _, encoded_img = cv2.imencode(".jpg", request)
    img = base64.b64encode(encoded_img).decode("utf-8")
    pipe.send(img)

def process2(pipe):
    img = pipe.recv()
    image_data = base64.b64decode(img)
    img = np.frombuffer(image_data, dtype=np.uint8)
    image = cv2.imdecode(img, cv2.IMREAD_COLOR)

    cv2.imwrite("result3.jpg", image)

if __name__ == "__main__":
    # Create a Pipe
    parent_conn, child_conn = Pipe()

    # Create two processes, each with its own function and the Pipe
    p1 = Process(target=process1, args=(child_conn,))
    p2 = Process(target=process2, args=(parent_conn,))

    # Start both processes
    p1.start()
    p2.start()
    
    # Wait for both processes to finish
    p1.join()
    p2.join()
