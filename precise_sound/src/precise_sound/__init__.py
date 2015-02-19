import pygame

# global constants
FREQ = 44100   # same as audio CD
BITSIZE = -16  # unsigned 16 bit
CHANNELS = 2   # 1 == mono, 2 == stereo
BUFFER = 1024  # audio buffer size in no. of samples
FRAMERATE = 30 # how often to check if playback has finished

class Sound:
    def __init__(self, filename):
        try:
            pygame.mixer.init(FREQ, BITSIZE, CHANNELS, BUFFER)
        except pygame.error, exc:
            print >>sys.stderr, "Could not initialize sound system: %s" % exc
            return

        self.sound = pygame.mixer.Sound(filename)
        self.clock = pygame.time.Clock()

    def play(self):
        try:
            self.sound.play()
            while pygame.mixer.get_busy():
                self.clock.tick(FRAMERATE)
        except KeyboardInterrupt:
            # if user hits Ctrl-C, exit gracefully
            return


