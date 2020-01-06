clear all

kuka=importrobot("iiwa14.urdf");
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

% Inverse Kinematics - Object Retrieval

ik = robotics.InverseKinematics('RigidBodyTree',kuka);
weights = [1 1 1 1 1 1];
initialguess = homeConfiguration(kuka);
[configSoln,solnInfo] = ik('iiwa_link_7',A_obj,weights,initialguess)


j1=configSoln(1).JointPosition
j2=configSoln(2).JointPosition
j3=configSoln(3).JointPosition
j4=configSoln(4).JointPosition
j5=configSoln(5).JointPosition
j6=configSoln(6).JointPosition
j7=configSoln(7).JointPosition