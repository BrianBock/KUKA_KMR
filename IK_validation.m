clearvars
kuka=importrobot("iiwa14.urdf");
ik = robotics.InverseKinematics('RigidBodyTree',kuka);
LastConfig = homeConfiguration(kuka);
ik.SolverParameters.MaxIterations=1500;

for j=1:15
    randConfig=randomConfiguration(kuka);
    t_form = getTransform(kuka,randConfig,'iiwa_link_7','iiwa_link_0');

    disp("Attempting to solve Inverse Kinematics for that pose...");
    weights = [1 1 1 1 1 1];
    initialguess = LastConfig;
    [LastConfig,solnInfo] = ik('iiwa_link_7',t_form,weights,initialguess);
    output=getTransform(kuka,LastConfig,'iiwa_link_7','iiwa_link_0');
    disp("Done");
    the_error=t_form/output
    the_error-eye(4)
end

