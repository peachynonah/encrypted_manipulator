Original Source: https://github.com/lsw23101/Encrypted_turtle.git 

Please check README.md.back, or attached github link for former information of encryption code.

# Overview
Code for encrypted control of two link robot manipulator.

Develop stage: 1. P controller 2. PD controller 3. PD+G controller

# P controller
P controller of 2 link manipulator requires multiplication of K gain (2d vector) and position error (2d vector).

Thus, modify scalar encryption method to vector encryption method is required.

Next, multiplication between enceypted vector is required.

**Currently on this stage.**

test: encrypt ee_pos = (x,y) position, p_gain = (constant, constant) and multiply encrypted data. 
