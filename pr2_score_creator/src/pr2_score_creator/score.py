class Score:
    def __init__(self, filedir, starting_scene):
        self.directory = filedir
        self.library = {}
        self.filename = None
        self.load(starting_scene)

    def load(self, filename, save=True):
        if save and self.filename is not None:
            self.to_file()
            self.library[self.filename] = self.movements

        self.filename = filename

        if os.path.exists(self.filename):
            self.movements = load_trajectory(self.filename)
        else:
            self.movements = []

    def is_valid_index(self, index=0):
        return len(self.movements) > index

    def to_file(self):
        print yaml.dump(self.movements)
        print
        save_trajectory(self.movements, self.filename)

    def to_full(self, filename=self.filename):
        full = []
        for move in load_trajectory(filename):
            if MACRO in move:
                key = move[MACRO]
                full += self.to_full("%s/%s.yaml"%(self.directory, key)
            else:
                full.append(move)
        return full
