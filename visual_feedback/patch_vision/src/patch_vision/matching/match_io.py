#!/usr/bin/env python

class Match:
    def __init__(self, compare_pt, reference_pt, strength, ref_idx=-1):
        self.compare_pt = compare_pt
        self.reference_pt = reference_pt
        self.strength = strength
        self.ref_idx = ref_idx

class MatchSet:
    def __init__(self, compare_image_path, reference_image_path):
        self.compare_image_path = compare_image_path
        self.reference_image_path = reference_image_path
        self.matches = []

    def add_match(self, compare_pt, reference_pt, strength, ref_idx = -1):
        self.matches.append( Match( compare_pt, reference_pt, strength, ref_idx) )

    def add_match(self, match):
        self.matches.append( match )

    def get_matches(self):
        return self.matches

    def save_to_file(self, filename):
        f = open(filename,'w')
        f.write("%d\n"%len(self.get_matches()))
        for match in self.get_matches():
            f.write("%f %f"%(match.compare_pt[0], match.compare_pt[1]))
            f.write("\t")
            f.write("%f %f"%(match.reference_pt[0], match.reference_pt[1]))
            f.write("\t")
            f.write("%f"%match.strength)
            f.write("\t")
            f.write("%d"%match.ref_idx)
            f.write("\n")
        f.close()

    def read_from_file(self, filename):
        f = open(filename, 'r')
        for i,ln in enumerate(f.readlines()):
            if i == 0:
                continue
            vals = [float(v) for v in ln.split()]
            compare_pt = (vals[0], vals[1])
            reference_pt = (vals[2], vals[3])
            strength = vals[4]
            ref_idx = -1 if len(vals) < 6 else int(ln.split()[5])
            self.matches.append( Match( compare_pt, reference_pt, strength, ref_idx ) )
        f.close()
        
        
