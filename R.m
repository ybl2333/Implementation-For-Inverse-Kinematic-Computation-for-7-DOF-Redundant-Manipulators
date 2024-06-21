%%% by FishCan
%%% Implementation for Analytical Inverse Kinematic Computation
%%% for 7-DOF Redundant Manipulators With Joint Limits
%%% and Its Application to Redundancy Resolution

function r=R(theta,alpha)
ct=cos(theta);
st=sin(theta);
ca=cos(alpha);
sa=sin(alpha);

r=[ct, -st*ca,  st*sa;
   st,  ct*ca, -ct*sa;
   0,   sa,     ca];
end