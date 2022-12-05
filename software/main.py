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
    TESTING = 0
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
    debug = True# if set to false wont show camera
    ball_to_right = False
    #camera instance for normal web cameras
    #cam = camera.OpenCVCamera(id = 2)
    # camera instance for realsense cameras
    cam = camera.RealsenseCamera(exposure = 100, depth_enabled=True)
    
    processor = image_processor.ImageProcessor(cam, debug=debug)

    processor.start()

    robot = motion.OmniMotionRobot()
    robot.open()

    ref_enabled = False
    if ref_enabled:
        robot_name = "teletupsud"
        referee = Referee_cmd_client()
        referee.open()
        current_state = State.STOPPED
    else:
        basket_color = "magenta"
        current_state = State.SEARCH_BALL

    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0
    cam_center = cam.rgb_width / 2
    center_offset = 20
    basket_offset = 8
    cam_lower_third = cam.rgb_height/3 * 2 + 20
    rotations = 0


    try:
        while True:
            if ref_enabled:
                msg = referee.get_cmd()
                if msg is not None:
                    if robot_name in msg["targets"]:
                        if msg["signal"] == "start":
                            # ref_first_start used if we want to differentiate between start of the match and resuming from a stop
                            if msg["baskets"][msg["targets"].index(robot_name)] == 'blue':
                                basket_color = "blue"
                            elif msg["baskets"][msg["targets"].index(robot_name)] == 'magenta':
                                basket_color = "magenta"
                            current_state = State.SEARCH_BALL
                        elif msg["signal"] == "stop":
                            current_state = State.STOPPED

            if current_state == State.STOPPED:
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


            print(current_state,ball_to_right,basket.x)


            if current_state == State.TESTING:
                if basket.x:
                    if abs(basket.x-cam_center)>basket_offset:
                        if basket.x <= cam_center - basket_offset:
                            robot.rotate(7)
                        elif basket.x >= cam_center + basket_offset:
                            robot.rotate(-7)
                        time.sleep(0.1)

                       
                    else:
                        print("tere")
                        thrower_speed = input("Sisesta thrower speed")
                        thrower_speed = int(thrower_speed)
                        time_x = time.time()
                        basket_x = basket.x
                        #print(basket_x)
                        while time.time() - time_x <= 0.3:
                            robot.throw(thrower_speed)
                        basket_distance = basket.distance
                        print(basket_distance,thrower_speed)
                continue


            elif current_state == State.FIND_BASKET:

                if (basket.x >= cam_center - basket_offset) and (basket.x <= cam_center + basket_offset):
                    current_state = State.THROW_BALL
                    #getting rid of motion blur
                    time.sleep(0.1)
                    continue
                
                # we check if ball is in middle every 0.8 sec
                elif time.time() - time_1 >= 0.8: 
                    current_state = State.DRIVE_TO_BALL
                    continue

                elif basket is not None:
                    rotation_speed = translate(basket.x, 0, cam.rgb_width, -50, 50)
                    rotation_speed = -rotation_speed
                    if 1>abs(rotation_speed)>=0:
                        if rotation_speed <= 0:
                            rotation_speed = -2
                        if rotation_speed > 0:
                            rotation_speed = 2
                    rotation_speed = int(rotation_speed)
                    robot.rotate(rotation_speed)
                    print(rotation_speed)
                
                elif basket is None:
                    # TODO try just rotate(20)
                    time_rotate = time.time()
                    while time.time() - time_rotate < 1:
                        robot.rotate(20)
                    time.sleep(0.05)
                continue


            elif current_state == State.DRIVE_TO_BASKET:
                if magenta_basket_dist > blue_basket_dist:
                    if processedData.basket_m.exists:
                        y = processedData.basket_m.y
                        x = processedData.basket_m.x
                        y_speed = (cam_lower_third - y) * 0.3 / 100
                        rot_speed = (cam_center - x) * Rot_constant / 100
                        robot.move(x_speed,y_speed,rot_speed)
                        if processedData.basket_m.distance <= 1700:
                            current_state = State.SEARCH_BALL
                        continue
                elif magenta_basket_dist <= blue_basket_dist:
                    if processedData.basket_b.exists:
                        y = processedData.basket_b.y
                        x = processedData.basket_b.x
                        y_speed = (cam_lower_third - y) * 0.3 / 100
                        rot_speed = (cam_center - x) * Rot_constant / 100
                        robot.move(x_speed,y_speed,rot_speed)
                        if processedData.basket_b.distance <= 1700:
                            current_state = State.SEARCH_BALL
                        continue
                robot.move(0,0,1)
                

            #kui korv on paremal või vasakul ta viskab mööda, alati
            elif current_state == State.THROW_BALL:
                x = processedData.balls[0].x
                #check if ball is in center
                if abs(x - cam_center) > center_offset:
                    current_state = State.DRIVE_TO_BALL
                    continue
                else:
                    basket_distance = basket.distance
                    basket_distance = basket_distance * 0.195 + 380
                    print(basket_distance,basket.x)
                    thrower_speed = int(basket_distance)
                    time_1 = time.time()
                    while time.time() - time_1 < 1 :
                        robot.throw(thrower_speed)
                    current_state = State.SEARCH_BALL
                    continue


            elif current_state == State.SEARCH_BALL:
                if processedData.balls:
                    current_state = State.DRIVE_TO_BALL
                    continue
                if rotations >= 5:
                    current_state = State.DRIVE_TO_BASKET
                    continue

                if processedData.basket_b.distance < blue_basket_dist:
                    blue_basket_dist = processedData.basket_b.distance
                if processedData.basket_m.distance < magenta_basket_dist:
                    magenta_basket_dist = processedData.basket_m.distance

                # TODO replace with just move
                time_now = time.time()
                our_delay = time_now - int(time_now)
                if our_delay <= 0.5:
                    if ball_to_right:
                        robot.move(0,0,1)
                    else:
                        robot.move(0,0,-1)
                    search_trig = True
                elif search_trig:
                    search_trig = False
                    rotations += 1
                    print(rotations)
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
