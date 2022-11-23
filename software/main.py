#import PYTHONPATH=${PYTHONPATH}:${HOME}/foo #meie path /home/robot/.local/lib/python3.9/site-packages/
import image_processor
import camera
import motion
import cv2
import time
import numpy as np
from enum import Enum

class State(Enum):
    TESTING = 0
    SEARCH_BALL = 1
    DRIVE_TO_BALL = 2
    FIND_BASKET = 3
    THROW_BALL = 4


def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)


def main_loop():
    debug = False# if set to false wont show camera
    ball_to_right = False
    
    #camera instance for normal web cameras
    #cam = camera.OpenCVCamera(id = 2)
    # camera instance for realsense cameras
    cam = camera.RealsenseCamera(exposure = 100, depth_enabled=True)
    
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
    basket_offset = 8
    cam_lower_third = cam.rgb_height/3 * 2 + 20

    current_state = State.SEARCH_BALL
    try:
        while True:
            processedData = processor.process_frame(aligned_depth=True)
            if debug:
                debug_frame = processedData.debug_frame

                cv2.imshow('debug', debug_frame)

                k = cv2.waitKey(1) & 0xff
                if k == ord('q'):
                    break
            print(current_state,ball_to_right,processedData.basket_b.x)
            if current_state == State.TESTING:
                if processedData.basket_b.x:
                    if abs(processedData.basket_b.x-cam_center)>basket_offset:
                        if processedData.basket_b.x <= cam_center - basket_offset:
                            robot.rotate(7)
                        elif processedData.basket_b.x >= cam_center + basket_offset:
                            robot.rotate(-7)
                        time.sleep(0.1)

                       
                    else:
                        print("tere")
                        thrower_speed = input("Sisesta thrower speed")
                        thrower_speed = int(thrower_speed)
                        time_x = time.time()
                        basket_x = processedData.basket_b.x
                        #print(basket_x)
                        while time.time() - time_x <= 0.3:
                            robot.throw(thrower_speed)
                        basket_distance = processedData.basket_b.distance
                        print(basket_distance,thrower_speed)
                continue
            if current_state == State.FIND_BASKET:

                if (processedData.basket_b.x >= cam_center - basket_offset) and (processedData.basket_b.x <= cam_center + basket_offset):
                    current_state = State.THROW_BALL
                    #getting rid of motion blur
                    time.sleep(0.1)
                    continue
                # we check if ball is in middle every 0.5 sec
                
                elif time.time() - time_1 >= 0.8: 
                    current_state = State.DRIVE_TO_BALL
                    continue

                elif processedData.basket_b is not None:
                    rotation_speed = translate(processedData.basket_b.x, 0, cam.rgb_width, -50, 50)
                    rotation_speed = -rotation_speed
                    if 1>abs(rotation_speed)>=0:
                        if rotation_speed <= 0:
                            rotation_speed = -2
                        if rotation_speed > 0:
                            rotation_speed = 2
                    rotation_speed = int(rotation_speed)
                    robot.rotate(rotation_speed)
                    print(rotation_speed)
                
                elif processedData.basket_b is None:
                    time_rotate = time.time()
                    while time.time() - time_rotate < 1:
                        robot.rotate(20)
                    time.sleep(0.05)
                continue

            #kui korv on paremal või vasakul ta viskab mööda, alati
            if current_state == State.THROW_BALL:
                x = processedData.balls[0].x
                #check if ball is in center
                if abs(x - cam_center) > center_offset:
                    current_state = State.DRIVE_TO_BALL
                    continue
                else:
                    basket_distance = processedData.basket_b.distance
                    basket_distance = basket_distance * 0.195 + 380
                    print(basket_distance,processedData.basket_b.x)
                    thrower_speed = int(basket_distance)
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
                time_search = time.time()
                while time.time() - time_search <= 0.6:
                    if ball_to_right:
                        robot.move(0,0,1)
                    else:
                        robot.move(0,0,-1)
                if ball_to_right:
                    robot.move(0,0,-0.3)
                else:
                    robot.move(0,0,0.3)
                time.sleep(0.5)
            
            elif current_state == State.DRIVE_TO_BALL:
                x = processedData.balls[0].x
                y = processedData.balls[0].y
                x_speed,y_speed,rot_speed = 0, 0, 0

                #Is the ball on the left or right?
                if abs(x - cam_center) > center_offset:
                    #print(f"koordinaat : {x}")
                    if y > cam_lower_third:
                        Rot_constant = 0.4
                    else: Rot_constant = 0.6
                    if x < cam_center - center_offset:
                        ball_to_right = True
                    else: 
                        ball_to_right = False
                    rot_speed = (cam_center - x) * Rot_constant / 100
                    
                else:
                    if y > cam_lower_third:
                        current_state = State.FIND_BASKET
                        time_1 = time.time()
                        continue
                #How far away is the ball
                if y < cam_lower_third:
                    y_speed = (cam_lower_third - y) * 0.3 / 100
                robot.move(x_speed,y_speed,rot_speed)

            frame_cnt +=1

            frame += 1
            if frame % 30 == 0:
                frame = 0
                end = time.time()
                fps = 30 / (end - start)
                start = end
                #print("FPS: {}, framecount: {}".format(fps, frame_cnt))
                #print("ball_count: {}".format(len(processedData.balls)))
                #if (frame_cnt > 1000):
                #    break

    except KeyboardInterrupt:
        print("closing....")
    finally:
        cv2.destroyAllWindows()
        processor.stop()

main_loop()
