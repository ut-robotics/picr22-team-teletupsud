#import PYTHONPATH=${PYTHONPATH}:${HOME}/foo #meie path /home/robot/.local/lib/python3.9/site-packages/
import image_processor
import camera
import motion
import cv2
import time
import numpy as np
from enum import Enum
from ref import RefereeClient
from referee import Referee_cmd_client

class State(Enum):
    SEARCH_BALL = 1
    DRIVE_TO_BALL = 2
    FIND_BASKET = 3
    THROW_BALL = 4
    STOPPED = 5
    DRIVE_TO_BASKET = 6


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

    ref_enabled = True
    if ref_enabled:
        robot_name = "teletupsud"
        referee = Referee_cmd_client()
        referee.open()
        current_state = State.STOPPED
    else:
        basket_color = "magenta"
        current_state = State.SEARCH_BALL

    #start = time.time()
    #fps = 0
    #frame = 0
    #frame_cnt = 0
    cam_center = cam.rgb_width / 2
    center_offset = 10
    basket_offset = 8
    cam_lower_third = cam.rgb_height/3 * 2 + 20

    rotations = 0
    blue_basket_dist = 4000 
    magenta_basket_dist = 4000
    rotation_done = False
    we_are_testing = False

    first_start = True

    try:
        while True:
            if ref_enabled:
                msg = referee.get_cmd()
                if msg is not None:
                    index = msg["targets"].index(robot_name) if robot_name in msg["targets"] else -1
                    if index != -1:
                        if msg["signal"] == "start":
                            basket_color = msg["baskets"][index]
                            if first_start:
                                if basket_color == "magenta":
                                    blue_basket_dist = 1
                                    magenta_basket_dist = 4000
                                else:
                                    blue_basket_dist = 4000
                                    magenta_basket_dist = 1
                                current_state = State.DRIVE_TO_BASKET
                                first_start = False
                            else:
                                current_state = State.SEARCH_BALL
                                
                        elif msg["signal"] == "stop":
                            current_state = State.STOPPED

            if current_state == State.STOPPED:
                robot.rotate(0)
                continue

            processedData = processor.process_frame(aligned_depth=True)

            if debug:
                debug_frame = processedData.debug_frame

                cv2.imshow('debug', debug_frame)

                k = cv2.waitKey(1) & 0xff
                if k == ord('q'):
                    break

            if basket_color == "blue":
                basket = processedData.basket_b
            elif basket_color == "magenta":
                basket = processedData.basket_m


            #print(current_state,ball_to_right,basket.x)

            if current_state == State.FIND_BASKET:

                if abs(basket.x-24 - cam_center) <= basket_offset:
                    current_state = State.THROW_BALL
                    robot.rotate(0)
                    basket_distance = basket.distance
                    if basket.distance > 2500:
                        basket_distance = 4069 -0.177 * basket.distance + 1.11 * 10 ** -4 * basket.distance ** 2
                    else:
                        basket_distance = 3093 +0.794 * basket.distance - 1.26 * 10 ** -4 * basket.distance ** 2 #0.182#0.351 * basket.distance + 3460#4069 -0.177 * basket.distance + 1.11 * 10 ** -4 * basket.distance ** 2 #0.182 * basket_distance  + 3806#346 + 0.164 * basket_distance + 9.82 * 10**-6 *basket_distance**2#555 + 6.05 *10**-3 * basket_distance + 4.79 * 10 **-5 * basket_distance**2#346 + 0.164 * basket_distance + 9.82 * 10**-6 *basket_distance**2
                    if we_are_testing:
                        print(basket.distance)
                        basket_distance = input("Sisessta soovitud kiirus :")
                    time_1 = time.time()
                    continue
                
                # we check if ball is in middle every 0.8 sec
                elif time.time() - time_1 >= 0.9: 
                    current_state = State.DRIVE_TO_BALL
                    continue

                elif basket is not None:
                    rotation_speed = translate(basket.x-24, 0, cam.rgb_width, -50, 50)
                    rotation_speed = -rotation_speed
                    if 2>abs(rotation_speed)>=0:
                        if rotation_speed <= 0:
                            rotation_speed = -2
                        if rotation_speed > 0:
                            rotation_speed = 2
                    rotation_speed = int(rotation_speed)
                    robot.rotate(rotation_speed)

                
                elif basket is None:
                    # maybe try just rotate(20)
                    time_rotate = time.time()
                    while time.time() - time_rotate < 1:
                        robot.rotate(20)
                    time.sleep(0.05)
                continue

            elif current_state == State.DRIVE_TO_BASKET:
                if processedData.basket_b.exists:
                    blue_basket_dist = processedData.basket_b.distance
                if processedData.basket_m.exists:
                    magenta_basket_dist = processedData.basket_m.distance
                current_basket = processedData.basket_m if magenta_basket_dist > blue_basket_dist else processedData.basket_b

                if current_basket.exists:
                    y = current_basket .y
                    x = current_basket .x
                    x_speed = 0
                    y_speed = (cam_lower_third - y) * 0.1 / 100
                    rot_speed = (cam_center - x) * 0.4 / 100
                    robot.move(x_speed,y_speed,rot_speed)
                    if current_basket.distance <= 2500:
                        current_state = State.SEARCH_BALL
                        rotations = 0
                        magenta_basket_dist = 4001
                        blue_basket_dist = 4000
                    continue
                robot.move(0,0,0.5)
                

            #kui korv on paremal või vasakul ta viskab mööda, alati
            elif current_state == State.THROW_BALL:
                if abs(x - cam_center) > center_offset:
                    current_state = State.DRIVE_TO_BALL
                    continue
                else:   
                    #snax 0.245 0.247 0.234 round(basketDistance*throwerMultiplier + 610) #695 #685 #300 #390
                    thrower_speed = int(basket_distance)
                    rotation_speed = translate(basket.x-24, 0, cam.rgb_width, -50, 50)
                    rotation_speed = -rotation_speed
                    #if 1>abs(rotation_speed)>=0:
                        #if rotation_speed <= 0:
                            #rotation_speed = -2
                        #if rotation_speed > 0:
                           # rotation_speed = 2
                    rotation_speed = int(rotation_speed)
                    #print(rotation_speed)
                    if time.time() - time_1 >= 0.5:
                        robot.throw(rotation_speed,thrower_speed)
                    if time.time() - time_1 >= 1.5 :
                        current_state = State.SEARCH_BALL
                    continue
                    


            elif current_state == State.SEARCH_BALL:
                if processedData.balls:
                    current_state = State.DRIVE_TO_BALL
                    continue
                if rotations >= 5:
                    current_state = State.DRIVE_TO_BASKET
                    continue
                
                if processedData.basket_b.exists:
                    if processedData.basket_b.distance < blue_basket_dist:
                        blue_basket_dist = processedData.basket_b.distance
                if processedData.basket_m.exists:
                    if processedData.basket_m.distance < magenta_basket_dist:
                        magenta_basket_dist = processedData.basket_m.distance

                time_now = time.time()
                our_delay = time_now - int(time_now)
                if our_delay <= 0.5:
                    if ball_to_right:
                        robot.move(0,0,1)
                    else:
                        robot.move(0,0,-1)
                    rotation_done = True
                elif rotation_done:
                    rotation_done = False
                    rotations += 1
                    if ball_to_right:
                        robot.move(0,0,-0.1)
                    else:
                        robot.move(0,0,0.1)
                    #time.sleep(0.5)
            
            elif current_state == State.DRIVE_TO_BALL:
                if not processedData.balls:
                    current_state = State.SEARCH_BALL
                    continue
                rotations = 0
                x = processedData.balls[0].x
                y = processedData.balls[0].y
                x_speed,y_speed,rot_speed = 0, 0, 0

                #Is the ball on the left or right?
                if abs(x - cam_center) > center_offset:
                    if y > cam_lower_third:
                        rot_constant = 0.4
                    else: rot_constant = 0.5
                    if x < cam_center - center_offset:
                        ball_to_right = True
                    else: 
                        ball_to_right = False
                    rot_speed = (cam_center - x) * rot_constant / 100
                    
                else:
                    if y >= cam_lower_third:
                        current_state = State.FIND_BASKET
                        time_1 = time.time()
                        continue
                #How far away is the ball
                if y < cam_lower_third:
                    y_speed = (cam_lower_third - y) * 0.4 / 100
                robot.move(x_speed,y_speed,rot_speed)


            #frame_cnt +=1
            #frame += 1
            #if frame % 30 == 0:
            #    frame = 0
            #    end = time.time()
            #    fps = 30 / (end - start)
            #    start = end
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
