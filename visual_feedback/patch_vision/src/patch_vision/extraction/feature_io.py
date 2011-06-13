#!/usr/bin/env python


class FeatureMap:
    def __init__(self):
        self.patch_size = None
        self.features = {}

    def add_feature(self, location, feature):
        self.features[location] = feature

    def get_feature(self, location):
        if not location in self.get_feature_points():
            raise Exception("That location has no associated feature")
        return self.features[location]

    def get_feature_points(self):
        return self.features.keys()

    def set_patch_size(self, patch_size):
        self.patch_size = patch_size

    def get_patch_size(self):
        return self.patch_size

    def save_to_file(self, filename):
        f = open(filename,'w')
        f.write("%d\n"%len(self.get_feature_points()))
        for pt in self.get_feature_points():
            f.write("%f %f"%(pt[0],pt[1]))
            f.write("\t")
            f.write("%f %f"%(self.patch_size, self.patch_size) )
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
            vals = [float(val_str) for val_str in ln.split()]
            pt = (vals[0],vals[1])
            patch_size = (vals[2], vals[3])
            self.set_patch_size(patch_size)
            num_features = vals[4];
            feature = vals[5:]
            assert len(feature) == num_features
            self.add_feature(pt, feature)
        f.close()

