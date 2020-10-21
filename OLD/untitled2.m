clear all; clc

axis([-3 3 -3 3 -2 5]);

mdl_twolink_sym
syms q1 q2 q1d q2d q1dd q2dd real
tau = twolink.rne([q1 q2], [q1d q2d], [q1dd q2dd]);

mdl_puma560