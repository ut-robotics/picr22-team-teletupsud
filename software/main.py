import image_processor
import camera
import motion
import cv2
import time

def main_loop():
    debug = True
    
    #motion_sim = motion.TurtleRobot()
    #motion_sim2 = motion.TurtleOmniRobot()
    
    #camera instance for normal web cameras
    #cam = camera.OpenCVCamera(id = 2)
    # camera instance for realsense cameras
    cam = camera.RealsenseCamera(exposure = 100)
    
    processor = image_processor.ImageProcessor(cam, debug=debug)

    processor.start()
    #motion_sim.open()
    #motion_sim2.open()

    robot = motion.OmniMotionRobot()
    serial_port = "/dev/ttyACM0"
    robot.open(serial_port)

    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0
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
                continue
            x = processedData.balls[0].x
            y = processedData.balls[0].y
            x_speed,y_speed,rot_speed = 0, 0, 0

            #Kas pall on vaskul voi paremal?
            if x< 424 or x>464:
                if x<404:
                    rot_speed = -4
                else: rot_speed = 4
            #Kui kaugel on pall
            if y<320:
                y_speed = 10

            #print(processedData.balls[0],rot_speed)
            #motion_sim.move(x_speed,y_speed,rot_speed)
            #motion_sim2.move(x_speed,y_speed,rot_speed)

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
        #motion_sim.close()
        #motion_sim2.close()

main_loop()
