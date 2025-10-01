cla;
robotModel = importrobot('C:\ChunWork\00. NAS\10. UST\2022-1 Introduction to Robotics\JuliaTest\UR5.urdf');
jointConfig = homeConfiguration(robotModel);

jAng = out.jAng.Data(:,:,1);
for j=1:6
    jointConfig(j).JointPosition = jAng(j);
end
show(robotModel,jointConfig,Visuals='on',Collisions='off',PreservePlot=0,FastUpdate=1);

%show(robotModel,Visuals='on',Collisions='off',PreservePlot=0);
hold on;
nSteps = length(out.tout);
for i=2:1:nSteps
    jAng = out.jAng.Data(:,:,i);
    dt = out.tout(i) - out.tout(i-1);
    for j=1:6
        jointConfig(j).JointPosition = jAng(j);
    end
    show(robotModel,jointConfig,Visuals='on',Collisions='off',PreservePlot=0,FastUpdate=1);
    drawnow;
    %pause(dt);
end