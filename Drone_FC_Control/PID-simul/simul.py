import numpy as np
import matplotlib
import turtle

#Global Params
TIMER = 0
SETPOINT = 10
SIM_TIME = 10

#---------------

class Simulation(object):
    def __init__(self):
        self.screen = turtle.Screen()
        self.screen.setup(1280, 900)
        self.marker = turtle.Turtle()
        self.marker.penup
        self.marker.left(90)
        self.marker.goto(15, SETPOINT)
    
    def cycle(self):        #Simulation Cycle
        while(self.sim):
            if self.timer > SIM_TIME:
                self.sim = False
            
#class Rocket(object):
    
        
def main():
    sim = Simulation()
    sim.cycle()
        
main()