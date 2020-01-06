%% Brian Bock and Shantam Bajpai
% ENPM662 - Modeling 
% Term Project
% Fall 2019

%% DH Table
clearvars
syms t1 t2 t3 t4 t5 t6 t7 d1 d2 d3 d4 d5 d6 d7

%Arbitrary values - change later
d1=2;
d2=2;
d3=2;
d4=2;
d5=2;
d6=2;
d7=2;


        % a alpha d theta
dhparams=[0  pi/2 d1 pi/2;
          0 -pi/2 d2 0;
          0 -pi/2 d3 0;
          0 pi/2 d4 0;
          0 pi/2 d5 0;
          0 -pi/2 d6 0;
          0 0 d7 -pi/2]
    


%% Creating The KUKA Manipulator
kuka = robotics.RigidBodyTree;

body1 = robotics.RigidBody('body1');
jnt1 = robotics.Joint('jnt1','revolute');
body2 = robotics.RigidBody('body2');
jnt2 = robotics.Joint('jnt2','revolute');
body3 = robotics.RigidBody('body3');
jnt3 = robotics.Joint('jnt3','revolute');
body4 = robotics.RigidBody('body4');
jnt4 = robotics.Joint('jnt4','revolute');
body5 = robotics.RigidBody('body5');
jnt5 = robotics.Joint('jnt5','revolute');
body6 = robotics.RigidBody('body6');
jnt6 = robotics.Joint('jnt6','revolute');
body7 = robotics.RigidBody('body7');
jnt7 = robotics.Joint('jnt7','revolute');

setFixedTransform(jnt1,dhparams(1,:),'dh');
setFixedTransform(jnt2,dhparams(2,:),'dh');
setFixedTransform(jnt3,dhparams(3,:),'dh');
setFixedTransform(jnt4,dhparams(4,:),'dh');
setFixedTransform(jnt5,dhparams(5,:),'dh');
setFixedTransform(jnt6,dhparams(6,:),'dh');
setFixedTransform(jnt7,dhparams(7,:),'dh');

body1.Joint = jnt1;
body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;
body7.Joint = jnt7;

addBody(kuka,body1,'base')
addBody(kuka,body2,'body1')
addBody(kuka,body3,'body2')
addBody(kuka,body4,'body3')
addBody(kuka,body5,'body4')
addBody(kuka,body6,'body5')
addBody(kuka,body7,'body6')

configuration = randomConfiguration(kuka)


show(kuka,configuration)

%% Object Retrieval Pose

%The object to retrieve is a disc on a spindle. The spindle is a distance
%l_obj along the (maipulator) Xo axis, h_obj along the (manipulator) Zo
%axis. It is then rotated by theta_obj about the Z axis, and by alpha_obj
%about the X axis. 
theta_obj=deg2rad(30);
h_obj=8;
l_obj=8;
alpha_obj=deg2rad(45);


Rotz_obj=[cos(theta_obj) sin(theta_obj)  0 0;
        -sin(theta_obj) cos(theta_obj) 0 0;
       0 0 1 0;
       0 0 0 1];
   
Transz_obj=[1 0 0 0;
         0 1 0 0;
         0 0 1 h_obj;
         0 0 0 1];
     
Transx_obj=eye(4);

Rotx_obj=[1 0 0 0;
       0 cos(alpha_obj) sin(alpha_obj) 0;
       0 -sin(alpha_obj) cos(alpha_obj) 0
       0 0 0 1];
   
   
A_obj=Rotz_obj*Transz_obj*Transx_obj*Rotx_obj

%% Inverse Kinematics - Object Retrieval

ik = robotics.InverseKinematics('RigidBodyTree',kuka);
weights = [1 1 1 1 1 1];
initialguess = randomConfiguration(kuka);
[configSoln,solnInfo] = ik('body7',A_obj,weights,initialguess)

show(kuka,configSoln)

%% Object Removal
%To remove the disc from the spindle, it must be slide a distance d_remove
%in the Z_obj direction

d_remove=3;

Rotz_remove=eye(4);
   
Transz_remove=[1 0 0 0;
         0 1 0 0;
         0 0 1 d_remove;
         0 0 0 1];
     
Transx_remove=eye(4);
Rotx_remove=eye(4);
A_remove=Rotz_remove*Transz_remove*Transx_remove*Rotx_remove;

A_0_remove=A_obj*A_remove

%% Inverse Kinematics - Object Removal

ik = robotics.InverseKinematics('RigidBodyTree',kuka);
weights = [1 1 1 1 1 1];
initialguess = randomConfiguration(kuka);
[configSoln,solnInfo] = ik('body7',A_0_remove,weights,initialguess)

show(kuka,configSoln)


%% 
clearvars