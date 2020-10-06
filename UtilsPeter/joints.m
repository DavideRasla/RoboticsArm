clear all; clc

a1 = 1; %first link
q1 = 0.2; %joint angle

%endEffector

EF = trchain2('R(q1) Tx(a1)', q1) %numeric rapresentation of the transformation matrix
                                  % representing the pose of the end effector

L = Link('revolute','d',1,2,'a', 0.3, 'alpha', pi/2);
