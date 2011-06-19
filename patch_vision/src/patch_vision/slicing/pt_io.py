#!/usr/bin/env python

class PointSet:
    def __init__(self):
       self.points = [] 

    def add_point(self, point):
        self.points.append(point)

    def get_points(self):
        return list(self.points)
        
    def get_point(self, index):
        return self.points[index]

    def num_points(self):
        return len(self.points)


    def save_to_file(self, filename):
        f = open(filename,'w')
        f.write("%d\n"%len(self.points))
        for pt in self.points:
            f.write("%f %f"%(pt[0],pt[1]))
            f.write("\n")
        f.close()

    def read_from_file(self, filename):
        f = open(filename,'r')
        for i,ln in enumerate(f.readlines()):
            if i == 0:
                continue
            vals = ln.split()
            pt = (float(vals[0]), float(vals[1]) )
            self.add_point( pt )
        f.close()


