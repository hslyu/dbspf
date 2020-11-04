#! /usr/bin/env python3

class node:
    def __init__(self, x=0, y=0, start_time=0, end_time):
        self.x = x
        self.y = y
        self.start_time = start_time
        self.end_time = end_time

class vehicle:
    def __init__(self, pos_x=0, pos_y=0, pos_h=0, velocity, service_time):
        self.pos_x=pos_x
        self.pos_y=pos_y
        self.pos_h=pos_z
        self.velocity=velocity
        self.service_time=service_time
    
