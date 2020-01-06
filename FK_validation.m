clearvars
%% DH Table
clearvars
syms t1 t2 t3 t4 t5 t6 t7 d1 d2 d3 d4 d5 d6 d7

%Arbitrary values
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

%% Testing
config = randomConfiguration(kuka)

t1=config(1).JointPosition;
t2=config(2).JointPosition;
t3=config(3).JointPosition;
t4=config(4).JointPosition;
t5=config(5).JointPosition;
t6=config(6).JointPosition;
t7=config(7).JointPosition;

    
t_form = getTransform(kuka,config,'body7','base')


%% Manual FK

%%%%%% A1 %%%%%%%%%
Rotz1=[cos(t1+pi/2) -sin(t1+pi/2) 0 0;
       sin(t1+pi/2) cos(t1+pi/2) 0 0;
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
   
   
A1=Rotz1*Transz1*Transx1*Rotx1;

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
   
   
A2=Rotz2*Transz2*Transx2*Rotx2;

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
   
   
A3=Rotz3*Transz3*Transx3*Rotx3;


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
   
   
A4=Rotz4*Transz4*Transx4*Rotx4;


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
   
   
A5=Rotz5*Transz5*Transx5*Rotx5;


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
   
   
A6=Rotz6*Transz6*Transx6*Rotx6;



%%%%%% A7 %%%%%%%%%

Rotz7=[cos(t7-pi/2) -sin(t7-pi/2) 0 0;
       sin(t7-pi/2) cos(t7-pi/2) 0 0;
       0 0 1 0;
       0 0 0 1];
   
Transz7=[1 0 0 0;
         0 1 0 0;
         0 0 1 d7;
         0 0 0 1];
     
Transx7=eye(4);

Rotx7=eye(4);
   
   
A7=Rotz7*Transz7*Transx7*Rotx7;


T=A1*A2*A3*A4*A5*A6*A7


