# InformedLeaders
3-D Model exploring the effect Informed Leaders have on large group movement

Main file that puts the simulation together is Final.c. Runs using Glew 2.2.0 and glfw 3.3.8. 
The makefile is designed for use with multiple architectures but only tested on an m1 mac. 
Simulation is based on the paper in files/.

# Keys
```  m     |  switch between modes
  a     |  toggle axis
arrows  |  rotate view
  k     |  pause simulation
  0     |  reset simulation
 1/!    |  increase/decrease a (minimum range radius)
 2/@    |  increase/decrease p (maximum sight radius)
 i/I    |  increase/decrease percentage of informed leaders
 w/W    |  increase/decrease informed leaders bias towards goal direction
 t/T    |  increase/decrease individual rotation speed
 r/R    |  increase/decrease individual velocity
```


 This model was created as a quick demonstration of the abiltiy of a small percentage of a population to steer group movements without any form of communication. 
 Uninformed individuals attempt to maintain a minimum radius between other individuals (a) and move towards individuals in their sight radius (r). Informed individuals
 also attempt to move towards a target direction, weighted by a constant w. In this model the target direction is vertex (1,1,1) and informed individuals are red. 
 
 

