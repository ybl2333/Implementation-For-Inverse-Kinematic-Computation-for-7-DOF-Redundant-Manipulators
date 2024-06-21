%%% by FishCan
%%% Implementation for Analytical Inverse Kinematic Computation
%%% for 7-DOF Redundant Manipulators With Joint Limits
%%% and Its Application to Redundancy Resolution

function qs=ikSolver(armAngle,target,d)
r07=[target.n target.o target.a];
x07=target.t;
qs=zeros(1,7);

d_bs=d(1);
d_se=d(2);
d_ew=d(3);
d_wt=d(4);

l_bs0=[0,0,d_bs]';
l_se3=[0,-d_se,0]';
l_ew4=[0,0,d_ew]';
l_wt7=[0,0,d_wt]';

x_sw=x07-l_bs0-r07*l_wt7;
d_sw=norm(x_sw);
qs(4)=acos((d_sw*d_sw-d_se*d_se-d_ew*d_ew)/(2*d_se*d_ew));

v_sw=l_se3+R(qs(4),pi/2)*l_ew4;
q10=atan2(x_sw(2),x_sw(1));
q20=asin(-x_sw(3)/sqrt(v_sw(1)*v_sw(1)+v_sw(2)*v_sw(2)))-atan2(v_sw(2),v_sw(1));

R030=R(q10,-pi/2)*R(q20,pi/2)*R(0,-pi/2);
u_sw=skew(x_sw/d_sw);

As=u_sw*R030;
Bs=-(u_sw*u_sw)*R030;
Cs=(eye(3)+u_sw*u_sw)*R030;

Aw=R(qs(4),pi/2)'*As'*r07;
Bw=R(qs(4),pi/2)'*Bs'*r07;
Cw=R(qs(4),pi/2)'*Cs'*r07;

tq1=(-As(2,2)*sin(armAngle)-Bs(2,2)*cos(armAngle)-Cs(2,2))/(-As(1,2)*sin(armAngle)-Bs(1,2)*cos(armAngle)-Cs(1,2));
cq2=-As(3,2)*sin(armAngle)-Bs(3,2)*cos(armAngle)-Cs(3,2);
tq3=(As(3,3)*sin(armAngle)+Bs(3,3)*cos(armAngle)+Cs(3,3))/(-As(3,1)*sin(armAngle)-Bs(3,1)*cos(armAngle)-Cs(3,1));

tq5=(Aw(2,3)*sin(armAngle)+Bw(2,3)*cos(armAngle)+Cw(2,3))/(Aw(1,3)*sin(armAngle)+Bw(1,3)*cos(armAngle)+Cw(1,3));
cq6=Aw(3,3)*sin(armAngle)+Bw(3,3)*cos(armAngle)+Cw(3,3);
tq7=(Aw(3,2)*sin(armAngle)+Bw(3,2)*cos(armAngle)+Cw(3,2))/(-Aw(3,1)*sin(armAngle)-Bw(3,1)*cos(armAngle)-Cw(3,1));

qs(1)=atan2((-As(2,2)*sin(armAngle)-Bs(2,2)*cos(armAngle)-Cs(2,2)),(-As(1,2)*sin(armAngle)-Bs(1,2)*cos(armAngle)-Cs(1,2)));
qs(2)=acos(cq2);
qs(3)=atan2((As(3,3)*sin(armAngle)+Bs(3,3)*cos(armAngle)+Cs(3,3)),(-As(3,1)*sin(armAngle)-Bs(3,1)*cos(armAngle)-Cs(3,1)));
qs(5)=atan2((Aw(2,3)*sin(armAngle)+Bw(2,3)*cos(armAngle)+Cw(2,3)),(Aw(1,3)*sin(armAngle)+Bw(1,3)*cos(armAngle)+Cw(1,3)));
qs(6)=acos(cq6);
qs(7)=atan2((Aw(3,2)*sin(armAngle)+Bw(3,2)*cos(armAngle)+Cw(3,2)),(-Aw(3,1)*sin(armAngle)-Bw(3,1)*cos(armAngle)-Cw(3,1)));
end