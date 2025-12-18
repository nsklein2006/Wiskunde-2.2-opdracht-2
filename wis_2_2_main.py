# -*- coding: utf-8 -*-
"""
Created on Mon Jul  3 14:33:49 2023

@author: Rik
"""

import wis_2_2_utilities as util
import wis_2_2_systems as systems
#import random
import numpy as np
import control as ct
#set timestep
timestep = 2e-3




class PID_controller():
  def __init__(self, target=0):
    self.K_P1=0.018
    self.K_I1=0
    self.K_D1=0.018
    
    self.K_P2=35
    self.K_I2=0
    self.K_D2=6
    
  def feedBack(self, observe):
    self.integral1=0
    self.integral2=0
    self.K_P1=0
    self.integral1+=observe[0]
    self.integral2+=observe[2]
    u=self.K_P1*observe[0]+\
      self.K_I1*self.integral1+\
      self.K_D1*observe[1]+\
      self.K_P2*observe[2]+\
      self.K_I2*self.integral2+\
      self.K_D2*observe[3]
    return u
  
#class pp_controller():
  #def __init__(self, target=0):
    #self.matrix_gain=np.array([[0, 0, 0, 0]])
    
  #def feedBack(self, observe):
    #u= -self.matrix_gain @ observe
    #return u  

class pp_controller():
    def __init__(self, target=0):
        matrix_A = np.array([[0,1,0,0],
        [21.0214, 0 , -21.0214, 0],
        [0,0,0,1],
        [-28.0286,0,77.0786,0]])
        matrix_B = np.array([[0],[28.3447],[0],[-70.8617]])
        
        print('the eigenvalues of A are:')
        print(np.linalg.eigvals(matrix_A))
        # zo ver mogelijk van elkaar af
        list_poles = [-5,-6,-7,-8] #6789
        matrix_K = ct.place(matrix_A,matrix_B,list_poles)
        self.matrix_gain = matrix_K

    def feedBack(self, observe):
        u = -self.matrix_gain @ observe
        return u  

class controller():
    def __init__(self, target=0):
        self.target = target
        
        self.Integral = 0
        self.K_P = -0.5
        self.K_I = -1
        self.K_D = -1
   
    def feedBack(self, observe):
        
        self.Integral += observe[0]*timestep
    
        #calculate feedback
        u=self.K_P*observe[0]+\
          self.K_I*self.Integral+\
          self.K_D*observe[1]
          
        return u
    
def main():
  model=systems.stacked_inverted_pendulum(num_pendulum = 2)
  control = pp_controller()
  simulation = util.simulation(model=model,timestep=timestep)
  simulation.setCost()
  #simulation.max_duration = 600 #seconde
  simulation.GIF_toggle = False #set to false to avoid frame and GIF creation

  while simulation.vis.Run():
      if simulation.time<simulation.max_duration:
        simulation.step()
        u = control.feedBack(simulation.observe())
        simulation.control(u)
        simulation.log()
        simulation.refreshTime()
      else:
        print('Ending visualisation...')
        simulation.vis.GetDevice().closeDevice()
        
  simulation.writeData()
 

  print("Input cost:", simulation.cost_input)       



if __name__ == "__main__":
  main()