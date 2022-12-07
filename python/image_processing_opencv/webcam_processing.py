import cv2
from collections import deque
import numpy as np
from term import term_utility
import time

class background_buffer:
    def __init__(self, width, height, maxlen):
        self.width = width
        self.height = height
        self.maxlen = maxlen

        self.buffer = deque(maxlen=maxlen)
        self.bg_frame = None

    def calculate_background(self):
        self.bg_frame = np.zeros((self.height, self.width), dtype="float32")

        for item in self.buffer:
            self.bg_frame += item
        self.bg_frame /= len(self.buffer)

    def update_background(self, old_frame, new_frame):
        self.bg_frame -= old_frame/self.maxlen
        self.bg_frame += new_frame/self.maxlen

    def update_frame(self, frame):
        if (len(self.buffer) == self.maxlen):
            old_frame = self.buffer.popleft()
            self.buffer.append(frame)
            self.update_background(old_frame, frame)
        else:
            self.buffer.append(frame)
            self.calculate_background()

    def get_background(self):
        return self.bg_frame.astype("uint8")


class motion_detection:
    def __init__(self, arduino_serial, cmd, width, height, down_scale, term):
        self.arduino_serial = arduino_serial
        self.cmd = cmd
        self.term = term

        # gaussian blur options
        self.gaus_blur_ksize_ds = (5,5)
        self.gaus_blur_sigmax_ds = 0

        # threshold() values for masks
        self.COLOR_VALUE_WHITE = 255 # Gray scale color white
        self.thresh_abs_mask_max = self.COLOR_VALUE_WHITE
        self.thresh_abs_mask_min = 15

        # contour options
        self.contour_thresh = 250
        self.COLOR_CODE_GREEN = (0, 255, 0)
        self.contour_rect_thickness = 2

        # buffer options
        self.bg_buff_len = 10

        self.width_ds = width // down_scale
        self.height_ds = height // down_scale
        self.frame_size_ds = (self.width_ds, self.height_ds)
        self.down_scale = down_scale

        self.bg_buffer = background_buffer(self.width_ds, self.height_ds,
                                           self.bg_buff_len)

    # args: frame, (width, height), bool flip, int mirror_flag
    def adjust_frame(self, frame, size, flip, mirror_flag = 1):
        new_frame = cv2.resize(frame, size)
        if (flip):
            new_frame = cv2.flip(new_frame, mirror_flag)
        return new_frame

    def get_gaus_gray(self, down_scale_frame, gaus_ksize, gaus_sig):
        gray = cv2.cvtColor(down_scale_frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, gaus_ksize, gaus_sig)
        return gray

    def add_to_buffer(self, buffer, frame):
        buffer.update_frame(frame)

    def abs_diff_threshold(self, thresh_frame, thresh_min, thresh_max,
                           thresh_type):
        abs_diff = cv2.absdiff(self.bg_buffer.get_background(), thresh_frame)

        retval, ad_mask = cv2.threshold(abs_diff, thresh_min, thresh_max,
                                         thresh_type)
        if (not retval):
            return retval, 0
        return retval, ad_mask


    def set_contours(self, frame, frame_mask, thresh, color, thickness,
                     down_scale):
        if (down_scale == 0):
            scale = 1
        else:
            scale = down_scale

        new_frame = frame
        # Define the contours
        contours, _ = cv2.findContours(frame_mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        # Set contour rectangles to the frame
        for contour in contours:
            # avoid small movements
            if (cv2.contourArea(contour) > thresh):
                x, y, w, h = cv2.boundingRect(contour)
                x, y, w, h = (x*scale, y*scale,
                              w*scale, h*scale)
                # Now draw the contour on the frame
                cv2.rectangle(new_frame, (x, y), (x + w, y + h),
                              color, thickness)

        return new_frame

    # If there no is motion, ad_mask will be completely black from threshholding
    # If there is motion, ad_mask will not be completely black
    # Set two ROI's one on the left and one on the right, ROI's come from mask
    #   If ROI is not black - we had movement in the frame
    #   If ROI is black - we had no movement in the frame
    #
    #   If there is movement on the right - return right flag as true
    #   If there is movement on the left - return left flag as true
    #   (Two return values, one left_movement, one right_movement)
    #
    #   If left flag is true, right flag is false, move braccio to position 1
    #   If right flag is true, left flag is false, move braccio to position 2
    #   If right and left flag is true, move braccio to position 3
    #
    #   If motion is detected, do not take into account further movement for
    #   a length of time so braccio can make it to the desired position and
    #   also to prevent 30 movement commands happening in a second. We probably
    #   want around 1 movement possible every 5-10 seconds depending on testing.
    #
    #   If no motion is detected, potentially for a specified ammount of time,
    #   go to default safe position
    #
    #   ISSUE: The webcam loop will happen, but when we enter read_exec to
    #          handle the command to the arduino, it waits until the command
    #          is finished so the webcam frames will not be pulled constantly
    #          like they should be. Multithreading can solve this but dont
    #          have time to figure out multithreading on python...
    #
    #          Might have to deal with this for now. Movement will happen in
    #          the frame -> send command to arduino --> wait for reply (time
    #          consuming) while no new frames are grabbed, webcam image on
    #          host will be frozen, --> grabes new frame, now any movement
    #          that occures during that wait for reply will cause the next
    #          frame to see movement due to the background frames not being
    #          updated.
    #
    #          Maybe update arduino firmware with a new message type that
    #          identifies its from the image processing webcam functionality
    #          and does not send information back to the host.
    #
    #          pySerial has reset_input_buffer() which we may be able to
    #          use instead of read_exec. Maybe after some time call
    #          reset_input_buffer so buffer doesnt overflow at some point, maybe
    #          after a specifid ammount of movement commands being sent.
    #          at the very least, not taking into account the chance of the
    #          buffer overflowing on the serial, call reset while exiting
    #          the webcam motion detection loop.
    #
    #
    #   UPDATE: Might try using read_exec with new flag in function to
    #           identify its being called during webcam execution so it doesnt
    #           block for user input. It might be fast enough during verbose
    #           printing. The issue is taking time to check ROI for so long
    #           seems not the best. Say the wait time is 15 sec, we cannot
    #           make the braccio move for the first initial 15 seconds of the
    #           execution which seems bad.
    #
    #           Further thinking, outside the initial say 15seconds, it will
    #           check the ROI at or over 15seconds, but if there is no movement
    #           it will not check again for another 15seconds. We would want it
    #           to check the ROI every time until there is movement. So if there
    #           is no movement we would want to keep checking the ROI each
    #           time until there is.
    #
    def detect_motion_moving_arm(self, frame, check_roi):
        FLIP = False
        movement = False
        left_move_pos = [180, 40, 180, 170, 90, 10] # angle positions for left
        right_move_pos = [90, 90, 90, 90, 90, 73] # angle positions for right

        down_scale_frame = self.adjust_frame(frame, self.frame_size_ds, FLIP)

        gray = self.get_gaus_gray(down_scale_frame, self.gaus_blur_ksize_ds,
                                  self.gaus_blur_sigmax_ds)

        self.add_to_buffer(self.bg_buffer, gray)

        retval, ad_mask = self.abs_diff_threshold(gray,
                                                  self.thresh_abs_mask_min,
                                                  self.thresh_abs_mask_max,
                                                  cv2.THRESH_BINARY)
        if (not retval):
            self.term.eprint("failed to get threshold mask for absolute diff")
            return False, movement, 0

        if (check_roi):
            # get ROI for left and right side of frame from ad_mask
            ad_mask_height, ad_mask_width = ad_mask.shape

            half_ad_mask_height = ad_mask_height // 2
            quarter_ad_mask_width = ad_mask_width // 4
            three_quarters_ad_mask_width = quarter_ad_mask_width * 3

            half_roi_height = 20
            half_roi_width = 20

            roi_row_start = half_ad_mask_height - half_roi_height
            roi_row_end = half_ad_mask_height + half_roi_height

            left_column_start = quarter_ad_mask_width - half_roi_width
            left_column_end = quarter_ad_mask_width + half_roi_width

            roi_left = ad_mask[roi_row_start : roi_row_end,
                               left_column_start : left_column_end]

            right_column_start = three_quarters_ad_mask_width - half_roi_width
            right_column_end = three_quarters_ad_mask_width + half_roi_width

            roi_right = ad_mask[roi_row_start : roi_row_end,
                                right_column_start : right_column_end]

            # See if there is any motion (any white in ROI section)
            roi_left_movement = roi_left.any()
            roi_right_movement = roi_right.any()
            if (roi_left_movement and roi_right_movement):
                self.term.print_verbose("BOTH SIDES MOVEMENT\n")
                msg = self.cmd.build_cmd_msg(self.cmd.SET_DFLT_POS)
                self.arduino_serial.write(msg)
                movement = True
            elif (roi_left_movement):
                self.term.print_verbose("LEFT SIDE MOVEMENT\n")
                msg = self.cmd.build_cmd_msg(self.cmd.MX_ANGLE,
                                             left_move_pos[0],
                                             left_move_pos[1],
                                             left_move_pos[2],
                                             left_move_pos[3],
                                             left_move_pos[4],
                                             left_move_pos[5],
                                             )
                self.arduino_serial.write(msg)
                movement = True
            elif (roi_right_movement):
                self.term.print_verbose("RIGHT SIDE MOVEMENT\n")
                msg = self.cmd.build_cmd_msg(self.cmd.MX_ANGLE,
                                             right_move_pos[0],
                                             right_move_pos[1],
                                             right_move_pos[2],
                                             right_move_pos[3],
                                             right_move_pos[4],
                                             right_move_pos[5],
                                             )
                self.arduino_serial.write(msg)
                movement = True
            else:
                self.term.print_verbose("NO MOVEMENT\n")
                movement = False

        # set contours on frame to see movement boxes
        new_frame = self.set_contours(frame,
                                      ad_mask,
                                      self.contour_thresh,
                                      self.COLOR_CODE_GREEN,
                                      self.contour_rect_thickness,
                                      self.down_scale)
        if (movement):
            self.arduino_serial.clear_input_buffer()

        return True, movement, new_frame

    def detect_motion(self, frame):
        FLIP = False

        down_scale_frame = self.adjust_frame(frame, self.frame_size_ds, FLIP)

        gray = self.get_gaus_gray(down_scale_frame, self.gaus_blur_ksize_ds,
                                  self.gaus_blur_sigmax_ds)

        self.add_to_buffer(self.bg_buffer, gray)

        retval, ad_mask = self.abs_diff_threshold(gray,
                                                  self.thresh_abs_mask_min,
                                                  self.thresh_abs_mask_max,
                                                  cv2.THRESH_BINARY)
        if (not retval):
            self.term.eprint("failed to get threshold mask for absolute diff")
            return False, 0

        # set contours on frame to see movement boxes
        new_frame = self.set_contours(frame,
                                      ad_mask,
                                      self.contour_thresh,
                                      self.COLOR_CODE_GREEN,
                                      self.contour_rect_thickness,
                                      self.down_scale)

        return True, new_frame

class webcam_processing():
    def __init__(self, arduino_serial, cmd, camera, width, height, down_scale,
                 fps, term):
        self.arduino_serial = arduino_serial
        self.cmd = cmd
        self.term = term
        self.frame_loss_limit = 20
        self.wait_key_delay = 1 # ms
        self.wait_key_delay_post_destroy_window = 1 # ms
        self.fps = fps
        self.fps_text_loc = (10, 30) # x,y, location on frame
        self.fps_font_scale = 2
        self.COLOR_CODE_BLUE = (255, 0, 0)
        self.fps_thickness = 2
        self.detect_fail_limit = 10

        self.WEBCAM_WINDOW_NAME = "Webcam"

        self.frame_width  = width # width of window when resizing/setting
        self.frame_height = height # height of window when resizing/setting
        self.frame_size   = (self.frame_width, self.frame_height)
        self.down_scale = down_scale

        self.cap = cv2.VideoCapture(camera, cv2.CAP_V4L2)
        if (not self.cap.isOpened()):
            self.term.eprint("Error: Could not open video capture")
            return 1

        # set fps
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        # set frame size
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        # init frame is a downscaled frame for motion detection
        self.motion_detect = motion_detection(arduino_serial, cmd, width,
                                              height, down_scale, term)

    def webcam_flow_with_braccio(self):
        MIRROR_FLAG = 1 # y-axis
        ROI_CHECK_TIME = 10 # seconds

        motion_detect = self.motion_detect

        count = 0
        fail_count = 0
        exit_flag = False
        fps_last_time = time.time()
        detect_last_time = time.time()
        first_roi_detect = False
        while (not exit_flag):
            retval, frame = self.cap.read()
            if (not retval):
                self.term.eprint("Error: Failed to read frame")
                count += 1
                if (count > self.frame_loss_limit):
                    self.term.eprint("Error: Lost too many frames in a row")
                    return 1
                else:
                    continue
            count = 0

            frame = cv2.resize(frame, self.frame_size)
            frame = cv2.flip(frame, MIRROR_FLAG)

            current_time = time.time()
            if (current_time - detect_last_time > ROI_CHECK_TIME or
                not first_roi_detect):
                retval, detect, frame = motion_detect.detect_motion_moving_arm(
                                                                        frame,
                                                                        True)
                if (detect):
                    detect_last_time = current_time
                    first_roi_detect = True
            else:
                retval, _, frame = motion_detect.detect_motion_moving_arm(frame,
                                                                          False)
            if (not retval):
                self.term.eprint("Failed to detect motion")
                fail_count += 1
                if (fail_count > self.detect_fail_limit):
                    self.term.eprint("Cannot detect motion, failing to "
                                     "proccess frames")
                    return 1
                else:
                    continue

            fail_count = 0

            # calc fps and show on frame
            current_time = time.time()
            fps_text = "FPS: " + str(1 // (current_time - fps_last_time))
            fps_last_time = current_time

            cv2.putText(frame, fps_text, self.fps_text_loc,
                        cv2.FONT_HERSHEY_PLAIN, self.fps_font_scale,
                        self.COLOR_CODE_BLUE, self.fps_thickness)

            cv2.imshow(self.WEBCAM_WINDOW_NAME, frame)

            # waitKey shows the actual window and waits for a key press
            if (cv2.waitKey(self.wait_key_delay) == ord('q')):
                cv2.destroyAllWindows()
                exit_flag = True

        self.arduino_serial.clear_input_buffer()
        return 0

    def webcam_flow_no_braccio(self):
        MIRROR_FLAG = 1 # y-axis

        motion_detect = self.motion_detect

        count = 0
        fail_count = 0
        exit_flag = False
        last_time = time.time()
        while (not exit_flag):
            retval, frame = self.cap.read()
            if (not retval):
                self.term.eprint("Error: Failed to read frame")
                count += 1
                if (count > self.frame_loss_limit):
                    self.term.eprint("Error: Lost too many frames in a row")
                    return 1
                else:
                    continue
            count = 0

            frame = cv2.resize(frame, self.frame_size)
            frame = cv2.flip(frame, MIRROR_FLAG)

            retval, frame = motion_detect.detect_motion(frame)
            if (not retval):
                self.term.eprint("Failed to detect motion")
                fail_count += 1
                if (fail_count > self.detect_fail_limit):
                    self.term.eprint("Cannot detect motion, failing to "
                                     "proccess frames")
                    return 1
                else:
                    continue

            fail_count = 0

            # calc fps and show on frame
            current_time = time.time()
            fps_text = "FPS: " + str(1 // (current_time - last_time))
            last_time = current_time

            cv2.putText(frame, fps_text, self.fps_text_loc,
                        cv2.FONT_HERSHEY_PLAIN, self.fps_font_scale,
                        self.COLOR_CODE_BLUE, self.fps_thickness)

            cv2.imshow(self.WEBCAM_WINDOW_NAME, frame)

            # waitKey shows the actual window and waits for a key press
            if (cv2.waitKey(self.wait_key_delay) == ord('q')):
                cv2.destroyAllWindows()
                exit_flag = True

        return 0
