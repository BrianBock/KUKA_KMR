%% Brian Bock
% ENPM662 - Modeling 
% Term Project
% Fall 2019

%% Joint 1
clearvars
syms t1 t2 t3 t4 d1 d2 d3 d4 t5 d5 t6 d6 d7 t7


%%%%%% A1 %%%%%%%%%
Rotz1=[cos(t1)*cosd(90)-sin(t1)*sind(90) -(sin(t1)*cosd(90)+cos(t1)*sind(90)) 0 0;
       (sin(t1)*cosd(90)+cos(t1)*sind(90)) cos(t1)*cosd(90)-sin(t1)*sind(90) 0 0;
       0 0 1 0;
       0 0 0 1];
   
Transz1=[1 0 0 0;
         0 1 0 0;
         0 0 1 d1;
         0 0 0 1];
     
Transx1=eye(4);

Rotx1=[1 0 0 0;
       0 cosd(90) -sind(90) 0;
       0 sind(90) cosd(90) 0
       0 0 0 1];
   
   
A1=Rotz1*Transz1*Transx1*Rotx1

%%%%%% A2 %%%%%%%%%

Rotz2=[cos(t2) -sin(t2) 0 0;
       sin(t2) cos(t2) 0 0;
       0 0 1 0;
       0 0 0 1];
   
Transz2=[1 0 0 0;
         0 1 0 0;
         0 0 1 d2;
         0 0 0 1];
     
Transx2=eye(4);

Rotx2=[1 0 0 0;
       0 cosd(-90) -sind(-90) 0;
       0 sind(-90) cosd(-90) 0
       0 0 0 1];
   
   
A2=Rotz2*Transz2*Transx2*Rotx2

%%%%%% A3 %%%%%%%%%

Rotz3=[cos(t3) -sin(t3) 0 0;
       sin(t3) cos(t3) 0 0;
       0 0 1 0;
       0 0 0 1];
   
Transz3=[1 0 0 0;
         0 1 0 0;
         0 0 1 d3;
         0 0 0 1];
     
Transx3=eye(4);

Rotx3=[1 0 0 0;
       0 cosd(-90) -sind(-90) 0;
       0 sind(-90) cosd(-90) 0
       0 0 0 1];
   
   
A3=Rotz3*Transz3*Transx3*Rotx3


%%%%%% A4 %%%%%%%%%

Rotz4=[cos(t4) -sin(t4) 0 0;
       sin(t4) cos(t4) 0 0;
       0 0 1 0;
       0 0 0 1];
   
Transz4=[1 0 0 0;
         0 1 0 0;
         0 0 1 d4;
         0 0 0 1];
     
Transx4=eye(4);

Rotx4=[1 0 0 0;
       0 cosd(90) -sind(90) 0;
       0 sind(90) cosd(90) 0
       0 0 0 1];
   
   
A4=Rotz4*Transz4*Transx4*Rotx4


%%%%%% A5 %%%%%%%%%

Rotz5=[cos(t5) -sin(t5) 0 0;
       sin(t5) cos(t5) 0 0;
       0 0 1 0;
       0 0 0 1];
   
Transz5=[1 0 0 0;
         0 1 0 0;
         0 0 1 d5;
         0 0 0 1];
     
Transx5=eye(4);

Rotx5=[1 0 0 0;
       0 cosd(90) -sind(90) 0;
       0 sind(90) cosd(90) 0
       0 0 0 1];
   
   
A5=Rotz5*Transz5*Transx5*Rotx5


%%%%%% A6 %%%%%%%%%

Rotz6=[cos(t6) -sin(t6) 0 0;
       sin(t6) cos(t6) 0 0;
       0 0 1 0;
       0 0 0 1];
   
Transz6=[1 0 0 0;
         0 1 0 0;
         0 0 1 d6;
         0 0 0 1];
     
Transx6=eye(4);

Rotx6=[1 0 0 0;
       0 cosd(-90) -sind(-90) 0;
       0 sind(-90) cosd(-90) 0
       0 0 0 1];
   
   
A6=Rotz6*Transz6*Transx6*Rotx6



%%%%%% A7 %%%%%%%%%

Rotz7=[cos(t7)*cosd(-90)-sin(t7)*sind(-90) -(sin(t7)*cosd(-90)+cos(t7)*sind(-90)) 0 0;
      (sin(t7)*cosd(-90)+cos(t7)*sind(-90)) cos(t7)*cosd(-90)-sin(t7)*sind(-90) 0 0;
       0 0 1 0;
       0 0 0 1];
   
Transz7=[1 0 0 0;
         0 1 0 0;
         0 0 1 d7;
         0 0 0 1];
     
Transx7=eye(4);

Rotx7=eye(4);
   
   
A7=Rotz7*Transz7*Transx7*Rotx7


T=A1*A2*A3*A4*A5*A6*A7

%% Jacobian 

o0=[0;0;0]
z0=[0;0;1]

o1=A1(1:3,4)
z1=A1(1:3,3)

A1A2=A1*A2;

o2=A1A2(1:3,4)
z2=A1A2(1:3,3)

A1A2A3=A1*A2*A3;
o3=A1A2A3(1:3,4)
z3=A1A2A3(1:3,3)

A1A2A3A4=A1*A2*A3*A4;
o4=A1A2A3A4(1:3,4)
z4=A1A2A3A4(1:3,3)

A1A2A3A4A5=A1*A2*A3*A4*A5;
o5=A1A2A3A4A5(1:3,4)
z5=A1A2A3A4A5(1:3,3)

A1A2A3A4A5A6=A1*A2*A3*A4*A5*A6;
o6=A1A2A3A4A5A6(1:3,4)
z6=A1A2A3A4A5A6(1:3,3)

A1A2A3A4A5A6A7=A1*A2*A3*A4*A5*A6*A7;
o7=A1A2A3A4A5A6A7(1:3,4)
z7=A1A2A3A4A5A6A7(1:3,3)

%cross(z1,(o7-o0))

Jv=[cross(z0,(o7-o0)) cross(z1,(o7-o1)) cross(z2,(o7-o2)) cross(z3,(o7-o3)) cross(z4,(o7-o4)) cross(z5,(o7-o5)) cross(z6,(o7-o6))]
Jw=[z0.*z0 z1.*z0 z2.*z0 z3.*z0 z4.*z0 z5.*z0 z6.*z0]

%Jacobian=[cross(z0,(o7-o0)) cross(z1,(o7-o1)) cross(z2,(o7-o2)) cross(z3,(o7-o3)) cross(z4,(o7-o4)) cross(z5,(o7-o5)) cross(z6,(o7-o6));
 %  z0.*z0 z1.*z0 z2.*z0 z3.*z0 z4.*z0 z5.*z0 z6.*z0];

%simplify(expand(Jacobian))


