function pose = PoseCevirici(xdot)
    persistent x 
    if isempty(x)
        x = 0; 
    end
    
    dt = 0.1; 
    x = x + (xdot * dt); 
    
    pose.ActorID = 1;
    pose.Position = [x, 0, 0];     
    pose.Velocity = [xdot, 0, 0];  
    pose.Roll = 0;
    pose.Pitch = 0;
    pose.Yaw = 0;                  
    pose.AngularVelocity = [0, 0, 0]; 
end