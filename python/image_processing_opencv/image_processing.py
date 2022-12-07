from webcam_processing import webcam_processing

class image_processing:
    def __init__(self, arduino_serial, cmd, kin, term, webcam_val=0,
                 resolution=(640,480), fps=30):
        self.arduino_serial = arduino_serial
        self.cmd = cmd
        self.kin = kin
        self.term = term
        self.dflt_camera = webcam_val
        self.width = resolution[0]
        self.height = resolution[1]
        self.down_scale = 2
        self.fps = fps

        self.webcam_exec = webcam_processing(self.arduino_serial, self.cmd,
                                             self.dflt_camera, self.width,
                                             self.height, self.down_scale,
                                             self.fps, self.term)

    def webcam_movement_no_braccio(self):
        err = self.webcam_exec.webcam_flow_no_braccio()
        if (err):
            self.term.eprint("Error on leaving webcam flow with no Braccio")
        return

    def webcam_movement_with_braccio(self):
        err = self.webcam_exec.webcam_flow_with_braccio()
        if (err):
            self.term.eprint("Error on leaving webcam flow with Braccio")

        # Return braccio to default position and update the kinematics class
        msg = self.cmd.build_cmd_msg(self.cmd.SET_DFLT_POS)
        self.arduino_serial.write(msg)
        self.cmd.read_exec()

        # update angles and displacement vectors
        msg = self.cmd.build_cmd_msg(self.cmd.REQUEST_MX_ANGLE)
        self.arduino_serial.write(msg)
        self.cmd.read_exec()

        self.kin.set_kin_vars()

        return
