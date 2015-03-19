from pr2_precise_trajectory import *
from pr2_precise_trajectory.converter import save_trajectory, load_trajectory
import os.path
import yaml
import rospy

class Score:
    def __init__(self, filename, filedir=None):
        self.directory = filedir
        self.library = {}
        self.filename = None
        self.load(filename)

    def load(self, filename, save=True):
        if save and self.filename is not None:
            self.to_file()
            self.library[self.filename] = self.movements

        self.filename = filename

        if os.path.exists(self.filename):
            self.movements = load_trajectory(self.filename)
        else:
            self.movements = []

    def save(self, index, state):
        if len(self.movements)==0:
            self.movements.append(state)
        elif not self.is_valid_index(index):
            return
        else:
            self.movements[index].update(state)

    def insert(self, after_index, state, ratio=0.5):
        index = after_index + 1
        n = len(self.movements)

        if index == n:
            # no element afterwards
            self.movements.append(state)
        elif self.is_valid_index(after_index):
            t = get_time( self.movements[index] )
            state[TIME] = t * ratio
            self.movements[index][TIME] = t * (1.0 - ratio)

            self.movements = self.movements[:index] + [state] + self.movements[index:]
        return index        

    def delete(self, index):
        if not self.is_valid_index(index):
            return None
        elif index == len(self.movements)-1:
            self.movements = self.movements[:index]
            return index - 1
        else:
            t =  get_time( self.movements[index]   )
            t2 = get_time( self.movements[index+1] )
            self.movements[index+1][TIME] = t + t2
            self.movements = self.movements[:index] + self.movements[index+1:]

    def set_property(self, index, field, value):
        if not self.is_valid_index(index):
            return

        m = self.movements[index]
        m[field] = value

    def get_property(self, index, field):
        if not self.is_valid_index(index):
            return
        m = self.movements[index]
        return m.get(field, None)

    def scale_time(self, index, factor, shift=True):
        """If shift is True, the movement 
            stays the same length """
        if not self.is_valid_index(index):
            return

        m = self.movements[index]
        t = get_time( m )
        new_t = t * factor

        if shift and index + 1 < len(self.movements):
            m2 = self.movements[index + 1]
            ot = get_time( m2 )
            dt = new_t - t
            if ot > dt:
                m[TIME] = new_t
                m2[TIME] = ot - dt
            else:
                rospy.logerr("TIME TOO SMALL")
        else:
            m[TIME] = new_t

    def total_time(self, i):
        return sum(get_time(m) for m in self.get_subset(end_i=i+1))

    def is_valid_index(self, index=0):
        return 0 <= index and index < len(self.movements)

    def has_data(self):
        return self.is_valid_index()

    def get_keyframe(self, index, backwards=False):
        if not self.is_valid_index(index):
            return {}

        m = {}
        m.update(self.movements[index])
        if backwards and index + 1< len(self.movements):
            m[TIME] = get_time(self.movements[index+1])
        # TODO: Check if need the specific subset
        #m2 = {}
        #for key in self.keys[1:]:
        #    if key in m:
        #        m2[key] = m[key]
        return m

    def get_state(self, index):
        if not self.is_valid_index(index):
            return {}
        ms = self.get_subset(end_i=index+1)
        return ms[0]

    def get_subset(self, start_i=0, end_i=None):
        return self.to_full(self.movements[start_i:end_i])

    def num_keyframes(self):
        return len(self.movements)

    def find_label(self, key):
        indexes = []
        for i, mv in enumerate(self.movements):
            label = mv.get('label', '')
            if label == key:
                indexes.append(i)
        return indexes

    def to_file(self):
        print yaml.dump(self.movements)
        print
        save_trajectory(self.movements, self.filename)

    def to_full(self, arg=None):
        if arg is None:
            ms = self.movements
        elif type(arg)==str:
            ms = load_trajectory(arg)
        else:
            ms = arg

        full = []
        for move in ms:
            if MACRO in move:
                key = move[MACRO]
                expanded = self.to_full("%s/%s.yaml"%(self.directory, key))
                if TIME in move:
                    expanded[0][TIME] = move[TIME]
                full += expanded
            else:
                full.append(move)
        return full
