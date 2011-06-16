#!/usr/bin/env python

SHAPES = ["SQUARE", "CIRCLE"]

class FeatureMap:
    def __init__(self):
        self.patch_size = None
        self.shapes = {}
        self.sizes = {}
        self.features = {}
        

    def add_feature(self, location, shape, size, feature):
        self.shapes[location] = shape
        self.sizes[location] = size
        self.features[location] = feature

    def get_shape(self, location):
        if not location in self.get_feature_points():
            raise Exception("That location has no associated feature")
        return self.shapes[location]
        
    def get_size(self, location):
        if not location in self.get_feature_points():
            raise Exception("That location has no associated feature")
        return self.sizes[location]
    
    def get_feature(self, location):
        if not location in self.get_feature_points():
            raise Exception("That location has no associated feature")
        return self.features[location]


    def get_feature_points(self):
        return self.features.keys()

    def save_to_file(self, filename):
        f = open(filename,'w')
        f.write("%d\n"%len(self.get_feature_points()))
        for pt in self.get_feature_points():
            f.write("%f %f"%(pt[0],pt[1]))
            f.write("\t")
            shape = self.get_shape(pt)
            f.write("%s"%shape)
            size = self.get_size(pt)
            f.write("%f %f"%(size[0], size[1]) )
            f.write("\t")
            feature = self.get_feature(pt)
            f.write( "%d "%len(feature))
            for val in feature:
                f.write("%f "%val)
            f.write("\n")
        f.close()

    def read_from_file(self, filename):
        f = open(filename,'r')
        for i,ln in enumerate(f.readlines()):
            if i == 0:
                continue
            str_vals = ln.split()
            pt = (float(str_vals[0]),float(str_vals[1]))
            shape = str_vals[2]
            patch_size = (float(str_vals[3]), float(str_vals[4]))
            num_features = float(str_vals[5]);
            feature = [float(val) for val in str_vals[6:]]
            assert len(feature) == num_features
            self.add_feature(pt, shape, patch_size, feature)
        f.close()

