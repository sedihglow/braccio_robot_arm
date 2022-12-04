from webcam_processing import webcam_processing

class image_processing:
    def __init__(self, term, webcam_val=0, resolution=(640,480), fps=30):
        self.term = term
        self.dflt_camera = webcam_val
        self.width = resolution[0]
        self.height = resolution[1]
        self.down_scale = 2
        self.fps = fps

        self.webcam_exec = webcam_processing(self.dflt_camera, self.width,
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
        return
