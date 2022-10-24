import image_processor
import camera
import motion
import cv2
import time
import numpy as np
from enum import Enum

class State(Enum):
    SEARCH_BALL = 1
    DRIVE_TO_BALL = 2
    FIND_BASKET = 3
    THROW_BALL = 4

def main_loop():
    debug = False# if set to false wont show camera
    ball_to_right = False
    
    #camera instance for normal web cameras
    #cam = camera.OpenCVCamera(id = 2)
    # camera instance for realsense cameras
    cam = camera.RealsenseCamera(exposure = 100, depth_enabled=False)
    
    processor = image_processor.ImageProcessor(cam, debug=debug)

    processor.start()

    robot = motion.OmniMotionRobot()
    robot.open()

    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0
    cam_center = cam.rgb_width / 2
    center_offset = 20
    cam_lower_third = cam.rgb_height/3 * 2

    current_state = State.SEARCH_BALL
    try:
        while True:
            processedData = processor.process_frame(aligned_depth=False)
            if debug:
                debug_frame = processedData.debug_frame

                cv2.imshow('debug', debug_frame)

                k = cv2.waitKey(1) & 0xff
                if k == ord('q'):
                    break

            if current_state == State.FIND_BASKET:
                robot.rotate()
                if (processedData.basket_b.x > cam_center - center_offset) and (processedData.basket_b.x < cam_center + center_offset):
                    current_state = State.THROW_BALL
                continue
            if current_state == State.THROW_BALL:
                thrower_speed = 500
                time_1 = time.time()
                while time.time() - time_1 < 1 :
                    robot.throw(thrower_speed)
                current_state = State.SEARCH_BALL
                continue
            
            if not processedData.balls:
                current_state = State.SEARCH_BALL
            else:
                current_state = State.DRIVE_TO_BALL


            if current_state == State.SEARCH_BALL:
                if ball_to_right:
                    robot.move(0,0,0.4)
                else: robot.move(0,0,-0.4)
            
            elif current_state == State.DRIVE_TO_BALL:
                x = processedData.balls[0].x
                y = processedData.balls[0].y
                x_speed,y_speed,rot_speed = 0, 0, 0

                #Is the ball on the left or right?
                if abs(x - cam_center) > center_offset:
                    if y > cam_lower_third:
                        Rot_constant = 0.1
                    else: Rot_constant = 0.2
                    if x < cam_center - center_offset:
                        ball_to_right = False
                    else: 
                        ball_to_right = True
                    rot_speed = (cam_center - x) * Rot_constant / 100
                    
                else:
                    if y > cam_lower_third:
                        current_state = State.FIND_BASKET
                        continue
                #How far away is the ball
                if y < cam_lower_third:
                    y_speed = (cam_lower_third - y) * 0.2 / 100
                robot.move(x_speed,y_speed,rot_speed)

            frame_cnt +=1

            frame += 1
            if frame % 30 == 0:
                frame = 0
                end = time.time()
                fps = 30 / (end - start)
                start = end
                print("FPS: {}, framecount: {}".format(fps, frame_cnt))
                print("ball_count: {}".format(len(processedData.balls)))
                #if (frame_cnt > 1000):
                #    break

    except KeyboardInterrupt:
        print("closing....")
    finally:
        cv2.destroyAllWindows()
        processor.stop()

main_loop()
