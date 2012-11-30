import math

def modify_linearize(trajectory, resolution=.1, arms=['l', 'r']):
    last_pt = trajectory[0] 
    new_trajectory = [ last_pt ]
    
    for move in trajectory[1:]:
        time = move.get('time', 3.0)
        num_points = int(math.floor(time / resolution))

        for a in range(num_points):
            frac = (a+1)*resolution / time
            np = {'time': resolution}
            for arm in arms:
                opos = last_pt[arm]
                pos = move[arm]
                npos = [x+(y-x)*frac for (x,y) in zip(opos, pos)]
                np[arm] = npos
            new_trajectory.append(np)

        move['time'] = time - num_points * resolution
        new_trajectory.append(move)
        last_pt = move

    return new_trajectory

def moving_average( data, W=1, B=2 ):
    avg = []

    for (i,x) in enumerate(data):
        start = max(0, i-W)
        end = min(i+W+1, len(data))
        total = 0.0
        divisor = 0.0
        
        for j in range(start, end):
            dist = abs(j-i)
            f = pow(B, -(dist))
            total += data[j] * f
            divisor += f

        x = total / divisor
        avg.append(x)
    return avg

def modify_moving_average(trajectory, arms=['l', 'r']):
    new_trajectory = []
    for tmove in trajectory:
        m = {}
        for k,v in tmove.iteritems():
            if k in arms:
                continue
            m[k] = v
        for arm in arms:
            m[arm] = []
        new_trajectory.append(m)

    for arm in arms:
        if arm not in trajectory[0]:
            continue
        for i in range(len(trajectory[0][arm])):
            data = [move[arm][i] for move in trajectory]
            filtered = moving_average(data)
            for t, nv in enumerate(filtered):
                new_trajectory[t][arm].append(nv)
    return new_trajectory

def modify_kalman(trajectory, arms=['l', 'r'], Q=0.0008, R=0.0025, P=0.001):
    new_trajectory = []
    p = {}
    x = {}
    for move in trajectory:
        j = {}
        for k,v in move.iteritems():
            if k not in arms:
                j[k] = v
                continue
            arm = k
            m = []
            for i in range(len(move[arm])):
                key = (arm, i)
                v = move[arm][i]
                if key not in p:
                    p[key] = P
                    x[key] = v
                    m.append(v)
                else:
                    p[key] += Q
                    k = p[key] / (p[key]+R)
                    x[key] = x[key] + k * (v - x[key])
                    p[key] = (1 - k) * p[key]
                    m.append(x[key])
            j[arm] = m
        new_trajectory.append(j)
    return new_trajectory
            
