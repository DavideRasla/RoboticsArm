clear all; clc
%{
mdl_planar2
p2.plot(qz)
T = transl(3, 4, 0);

%inverse Kin

q = p2.ikine(T, [1, 1],'mask', [1, 1, 0, 0, 0, 0] );
p2.plot(q)


mdl_puma560
hold on
p560.plot(qz)

 T2 = transl(0.5, 0.2, 0) * rpy2tr(0, 180, 180, 'deg');
 q = p560.ikine(T2, [0 0 0 0 0 0]);
 q = p560.ikine(T2, [1, 1],'mask', [1, 1, 0, 0, 0, 0] );
%}
 pause;
 mdl_puma560
 p560.plot(qz);
   
 TA = transl(0.4, 0.2, 0) * trotx(pi);
 TB = transl(0.4, -0.2, 0) * trotx(pi/2)
 qA = p560.ikine6s(TA);
 qB = p560.ikine6s(TB);
 
 tg = jtraj(qA, qB, 50);
 p560.plot(tg);
 


