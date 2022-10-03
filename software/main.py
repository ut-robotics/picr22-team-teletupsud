import image_processor
import camera
import motion
import cv2
import time
import numpy as np

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

    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0
    cam_width_Right, cam_width_Left = cam.rgb_width / 2 + 20, cam.rgb_width / 2 - 20
    cam_lower_third = cam.rgb_height/3 * 2
    try:
        while True:
            # has argument aligned_depth that enables depth frame to color frame alignment. Costs performance
            processedData = processor.process_frame(aligned_depth=False)
            if debug:
                debug_frame = processedData.debug_frame

                cv2.imshow('debug', debug_frame)

                k = cv2.waitKey(1) & 0xff
                if k == ord('q'):
                    break

            if not processedData.balls:
                if ball_to_right:
                    robot.move(0,0,0.4)
                else: robot.move(0,0,-0.4)
                continue
            x = processedData.balls[0].x
            y = processedData.balls[0].y
            x_speed,y_speed,rot_speed = 0, 0, 0

            #Is the ball on the left or right?
            if x < cam_width_Left or x > cam_width_Right:
                if x < cam_width_Left:
                    ball_to_right = False
                    rot_speed = (cam_width_Left - x) * -0.2 / 100
                else: 
                    ball_to_right = True
                    rot_speed = (cam_width_Right - x) * 0.2 / 100
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
