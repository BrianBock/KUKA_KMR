%% Project Setup
clc
clear all
global vrep;
global clientID;
global wheel_separation_width;
global wheel_radius;
global wheel_separatation_length;
global KMR_Front_Right_Wheel;
global KMR_Front_Left_Wheel;
global KMR_Rear_Right_Wheel;
global KMR_Rear_Left_Wheel;
global KUKA_Joint1;
global KUKA_Joint2;
global KUKA_Joint3;
global KUKA_Joint4;
global KUKA_Joint5;
global KUKA_Joint6;
global KUKA_Joint7;
global ik;
global kuka;
global LastConfig;
global gripper;
global gripper_connect;

wheel_separation_width=.5/2;%m
wheel_radius=2.5400E-01; %meters
wheel_separatation_length=.625/2; %m


kuka=importrobot("iiwa14.urdf");
ik = robotics.InverseKinematics('RigidBodyTree',kuka);
initialguess = homeConfiguration(kuka);
LastConfig=initialguess;
ik.SolverParameters.MaxIterations=1500;
%% Connect to VRep

vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);


if (clientID<=-1) % If not true, we've connected
    disp('Cannot connect to VRep. Make sure the Vrep simulation is already running');
    return;
end

if(clientID>-1)
disp('Connected to vrep. Ready to go!');
end

%% Get Object Handles for all Parts

[~, KMR_Front_Right_Wheel]=vrep.simxGetObjectHandle(clientID, 'Omnirob_FRwheel_motor', vrep.simx_opmode_blocking);
[~, KMR_Front_Left_Wheel]=vrep.simxGetObjectHandle(clientID, 'Omnirob_FLwheel_motor', vrep.simx_opmode_blocking);
[~, KMR_Rear_Right_Wheel]=vrep.simxGetObjectHandle(clientID, 'Omnirob_RRwheel_motor', vrep.simx_opmode_blocking); 
[~, KMR_Rear_Left_Wheel]=vrep.simxGetObjectHandle(clientID, 'Omnirob_RLwheel_motor', vrep.simx_opmode_blocking);
[~, KUKA_Joint1]=vrep.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint1', vrep.simx_opmode_blocking);
[~, KUKA_Joint2]=vrep.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint2', vrep.simx_opmode_blocking);
[~, KUKA_Joint3]=vrep.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint3', vrep.simx_opmode_blocking);
[~, KUKA_Joint4]=vrep.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint4', vrep.simx_opmode_blocking);
[~, KUKA_Joint5]=vrep.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint5', vrep.simx_opmode_blocking);
[~, KUKA_Joint6]=vrep.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint6', vrep.simx_opmode_blocking);
[~, KUKA_Joint7]=vrep.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint7', vrep.simx_opmode_blocking);
[~, gripper]=vrep.simxGetObjectHandle(clientID, 'RG2_openCloseJoint', vrep.simx_opmode_blocking);
[~, gripper_connect]=vrep.simxGetObjectHandle(clientID, 'RG2_attachPoint', vrep.simx_opmode_blocking);
[~, item]=vrep.simxGetObjectHandle(clientID, 'Box', vrep.simx_opmode_blocking);


%% Drive the Cart
%pause(10);
drive(11.5,5);
rotate(100);
drive(15,5);
rotate(-100);
drive(9.5,5);
rotate(-100);
drive(15,5);
rotate(100);
drive(3,-5);



%% Object Retrieval Pose

%The object to retrieve is a disc on a spindle. The spindle is a distance
%l_obj along the (manipulator) Xo axis, h_obj along the (manipulator) Zo
%axis. It is then rotated by theta_obj about the Z axis, and by alpha_obj
%about the X axis. 

theta_obj=deg2rad(30);
h_obj=.25;
l_obj=-.75;
alpha_obj=deg2rad(210);


Rotz_obj=[cos(theta_obj) -sin(theta_obj)  0 0;
        sin(theta_obj) cos(theta_obj) 0 0;
       0 0 1 0;
       0 0 0 1];

Transz_obj=[1 0 0 0;
         0 1 0 0;
         0 0 1 h_obj;
         0 0 0 1];

Transx_obj=[1 0 0 l_obj;
            0 1 0 0;
            0 0 1 0;
            0 0 0 1];
        
Rotx_obj=[1 0 0 0;
       0 cos(alpha_obj) -sin(alpha_obj) 0;
       0 sin(alpha_obj) cos(alpha_obj) 0
       0 0 0 1];


A_obj=Rotz_obj*Transz_obj*Transx_obj*Rotx_obj;


movearm(A_obj)
pause(2)

grip(item,1);

%% Object Removal
%To remove the disc from the spindle, it must be slide a distance d_remove
%in the Z_obj direction

dz_remove=-.25;
dx_remove=0;

Rotz_remove=eye(4);
   
Transz_remove=[1 0 0 0;
               0 1 0 0;
               0 0 1 dz_remove;
               0 0 0 1];
     
Transx_remove=[1 0 0 dx_remove;
               0 1 0 0;
               0 0 1 0;
               0 0 0 1];
           
Rotx_remove=eye(4);
A_remove=Rotz_remove*Transz_remove*Transx_remove*Rotx_remove;

A_0_remove=A_obj*A_remove;

%show(kuka,A_0_remove)

%movearm(A_0_remove)

%movearm(randomConfiguration(kuka));


drive(3,5)
rotate(-100);
drive(15,-5)
rotate(100);
drive(19,-5);

rotate(-100);
drive(15,5);
rotate(100);

grip(item,0);






vrep.simxFinish(-1); % Close the connection


    
vrep.delete(); % Call the vrep destructor




%% Functions

function driver=drive(distance, speed)% Drive Forward (direction =1), Reverse (direction=-1)
global clientID;
global vrep;
global KMR_Front_Right_Wheel;
global KMR_Front_Left_Wheel;
global KMR_Rear_Right_Wheel;
global KMR_Rear_Left_Wheel;

if(speed>0)
    disp("Driving Foward!");
elseif(speed<0)
    disp("Beep beep beep - driving backwards!");
end
        wheel_velocity=speed;
        duration=abs(distance/(.3*speed));
        %spin wheels
        [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Front_Left_Wheel,wheel_velocity,vrep.simx_opmode_blocking);
        [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Front_Right_Wheel,-wheel_velocity,vrep.simx_opmode_blocking);
        [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Rear_Right_Wheel,-wheel_velocity,vrep.simx_opmode_blocking);
        [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Rear_Left_Wheel,wheel_velocity,vrep.simx_opmode_blocking);
        pause(duration)
        %stop wheels
        [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Front_Left_Wheel,0,vrep.simx_opmode_blocking);
        [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Front_Right_Wheel,0,vrep.simx_opmode_blocking);
        [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Rear_Left_Wheel,0,vrep.simx_opmode_blocking);
        [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Rear_Right_Wheel,0,vrep.simx_opmode_blocking);
        pause(.5);
end


% function driver=drive(wheel_velocity,duration,direction)% Drive
% Forward (direction =1), Reverse (direction=-1)
%Original drive function, uses wheel_speed and time
% global clientID;
% global vrep;
% global KMR_Front_Right_Wheel;
% global KMR_Front_Left_Wheel;
% global KMR_Rear_Right_Wheel;
% global KMR_Rear_Left_Wheel;
% 
% if(direction==1)
%     disp("Driving Foward!");
% elseif(direction==-1)
%     disp("Beep beep beep - driving backwards!");
% end
%         wheel_velocity=wheel_velocity*direction;
%         %spin wheels
%         [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Front_Left_Wheel,wheel_velocity,vrep.simx_opmode_blocking);
%         [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Front_Right_Wheel,-wheel_velocity,vrep.simx_opmode_blocking);
%         [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Rear_Right_Wheel,-wheel_velocity,vrep.simx_opmode_blocking);
%         [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Rear_Left_Wheel,wheel_velocity,vrep.simx_opmode_blocking);
%         pause(duration)
%         %stop wheels
%         [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Front_Left_Wheel,0,vrep.simx_opmode_blocking);
%         [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Front_Right_Wheel,0,vrep.simx_opmode_blocking);
%         [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Rear_Left_Wheel,0,vrep.simx_opmode_blocking);
%         [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Rear_Right_Wheel,0,vrep.simx_opmode_blocking);
% end


function pivot=rotate(angle) % Rotate about central axis (deg)
    global clientID;
    global vrep;
    global KMR_Front_Right_Wheel;
    global KMR_Front_Left_Wheel;
    global KMR_Rear_Right_Wheel;
    global KMR_Rear_Left_Wheel;
    global wheel_radius;
    global wheel_separation_width
    global wheel_separatation_length
    angle=deg2rad(angle);
    
    disp("Turning!");
    % https://robohub.org/drive-kinematics-skid-steer-and-mecanum-ros-twist-included/
    v_wheel_front_left = -(1/wheel_radius)*((wheel_separation_width + wheel_separatation_length)*angle);
    v_wheel_front_right = -(1/wheel_radius)*((wheel_separation_width + wheel_separatation_length)*angle);
    v_wheel_rear_left = -(1/wheel_radius)*((wheel_separation_width + wheel_separatation_length)*angle);
    v_wheel_rear_right = -(1/wheel_radius)*((wheel_separation_width + wheel_separatation_length)*angle);
    
    %spin wheels
    [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Front_Left_Wheel,v_wheel_front_left,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Front_Right_Wheel, v_wheel_front_right,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Rear_Right_Wheel,v_wheel_rear_left,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Rear_Left_Wheel,v_wheel_rear_right,vrep.simx_opmode_blocking);
    pause(1)
    %stop wheels
    [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Front_Left_Wheel,0,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Front_Right_Wheel,0,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Rear_Left_Wheel,0,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetVelocity(clientID,KMR_Rear_Right_Wheel,0,vrep.simx_opmode_blocking);
    pause(.5);
end








function movearm=movearm(pose)
    global vrep;
    global kuka;
    global ik;
    global clientID;
    global KUKA_Joint1;
    global KUKA_Joint2;
    global KUKA_Joint3;
    global KUKA_Joint4;
    global KUKA_Joint5;
    global KUKA_Joint6;
    global KUKA_Joint7; 
    global LastConfig;
    disp("Attempting to solve Inverse Kinematics for that pose...");
    weights = [10 10 10 10 10 10];
    initialguess = LastConfig;
    [LastConfig,solnInfo] = ik('iiwa_link_7',pose,weights,initialguess)
    disp("Done solving inverse kinematics; Ready to move!");
    

    j1=LastConfig(1).JointPosition;
    j2=LastConfig(2).JointPosition;
    j3=LastConfig(3).JointPosition;
    j4=LastConfig(4).JointPosition;
    j5=LastConfig(5).JointPosition;
    j6=LastConfig(6).JointPosition;
    j7=LastConfig(7).JointPosition;

    disp("Moving robot arm");
    
    [~]=vrep.simxSetJointTargetPosition(clientID,KUKA_Joint1,j1,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetPosition(clientID,KUKA_Joint2,j2,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetPosition(clientID,KUKA_Joint3,j3,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetPosition(clientID,KUKA_Joint4,j4,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetPosition(clientID,KUKA_Joint5,j5,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetPosition(clientID,KUKA_Joint6,j6,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetPosition(clientID,KUKA_Joint7,j7,vrep.simx_opmode_blocking);
    
    disp("Done moving");
    show(kuka,LastConfig);
end




function grabber=grip(obj,grab) %obj is the item to grab (handle) and grab is 1 (Grab) or 0 (let go)
    global clientID
    global gripper_connect
    global vrep;
    motorVelocity=0.05 %-- m/s
    motorForce=20 %-- N
    item=obj;
    
    if(grab==1) %grab it!
        [~]=vrep.simxSetObjectParent(clientID,item,gripper_connect,true,vrep.simx_opmode_blocking)
        disp('Grabbing!');
    elseif(grab==0)  %let go
        [~]=vrep.simxSetObjectParent(clientID,item,-1,true,vrep.simx_opmode_blocking)
        disp('Dropping!');
    end
    
    % [~]=vrep.simxSetJointForce(clientID,gripper,motorForce,vrep.simx_opmode_blocking)
    % [~]=vrep.simxSetJointTargetVelocity(clientID,gripper,-motorVelocity,vrep.simx_opmode_blocking)
    

% if (vrep.simxGetObjectInt32Parameter(shape,sim.shapeintparam_static)==0) and (sim.getObjectInt32Parameter(shape,sim.shapeintparam_respondable)~=0) and (sim.checkProximitySensor(objectSensor,shape)==1) then
% %Ok, we found a non-static respondable shape that was detected
% attachedShape=shape
% %Do the connection:
% end

% ?-- You have basically 2 alternatives to grasp an object:
%     --
%     -- 1. You try to grasp it in a realistic way. This is quite delicate and sometimes requires
%     --    to carefully adjust several parameters (e.g. motor forces/torques/velocities, friction
%     --    coefficients, object masses and inertias)
%     --
%     -- 2. You fake the grasping by attaching the object to the gripper via a connector. This is
%     --    much easier and offers very stable results.
%     --
%     -- Alternative 2 is explained hereafter:
%     --
%     --
%     -- a) In the initialization phase, retrieve some handles:
%     -- 
%     -- connector=sim.getObjectHandle('RG2_attachPoint')
%     -- objectSensor=sim.getObjectHandle('RG2_attachProxSensor')
%     
%     -- b) Before closing the gripper, check which dynamically non-static and respondable object is
%     --    in-between the fingers. Then attach the object to the gripper:
%     --
%     -- index=0
%     -- while true do
%     --     shape=sim.getObjects(index,sim.object_shape_type)
%     --     if (shape==-1) then
%     --         break
%     --     end
%     --     if (sim.getObjectInt32Parameter(shape,sim.shapeintparam_static)==0) and (sim.getObjectInt32Parameter(shape,sim.shapeintparam_respondable)~=0) and (sim.checkProximitySensor(objectSensor,shape)==1) then
%     --         -- Ok, we found a non-static respondable shape that was detected
%     --         attachedShape=shape
%     --         -- Do the connection:
%     --         sim.setObjectParent(attachedShape,connector,true)
%     --         break
%     --     end
%     --     index=index+1
%     -- end
%     
%     -- c) And just before opening the gripper again, detach the previously attached shape:
%     --
%     -- sim.setObjectParent(attachedShape,-1,true)

end


