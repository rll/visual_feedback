#!/usr/bin/env python

import re

class LabelSet:
    def __init__(self):
        self.clear()
    
    def clear(self):
        self.label_colors = {}
        self.label_names = {}

    def add_label(self, label, name, color):
        #Only accept integer labels and RGB colors
        if not isinstance(label, (int,long) ) or label <= 0:
            raise Exception("Label must be a positive integer.")
        if re.search("\s",name):
            raise Exception("Label name cannot contain white space")
        self.label_names[label] = name
        #Convert from cv.BGR to regular color
        if len(color) == 4:
            label_color = (color[2],color[1],color[0])
        else:
            label_color = (color[0],color[1],color[2])
        self.label_colors[label] = label_color

    def get_labels(self):
        return self.label_colors.keys()

    def get_label_name(self, label):
        if label in self.label_names.keys():
            return self.label_names[label]

    def get_label_color(self, label):
        if label in self.label_colors.keys():
            return self.label_colors[label]

    def save_to_file(self,filename):
        f = open(filename,'w')
        f.write("Label\tName\t\tRGB\n")
        for label in self.get_labels():
            label_name = self.get_label_name(label)
            label_color = self.get_label_color(label)
            f.write("%d\t%s\t%03d %03d %03d\n"%
                    (label, label_name, 
                     label_color[0], label_color[1], label_color[2]))
        f.close()

    def read_from_file(self,filename):
        f = open(filename,'r')
        self.clear()
        for i,ln in enumerate(f.readlines()):
            if i == 0:
                continue
            tokens = ln.split()
            label = int(tokens[0])
            label_name = tokens[1]
            label_color = (int(tokens[2]), int(tokens[3]), int(tokens[4]) )
            self.label_names[label] = label_name
            self.label_colors[label] = label_color
        f.close()

