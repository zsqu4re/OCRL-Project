clear;

L0s=0.06:0.01:0.14; % L0变化范围
Ks=zeros(2,6,length(L0s)); % 存放不同L0对应的K

for step=1:length(L0s)

% Required symbolic variables
 syms theta theta1 theta2; % theta1=dTheta, theta2=ddTheta
 syms x x1 x2;
 syms phi phi1 phi2;
 syms T Tp N P Nm Pm Nf t;
% Robot structure parameters
 R=0.051; 
 L=L0s(step)/2; 
 Lm=L0s(step)/2; 
 l=0; 
 mw=0.078; 
 mp=0.0248; 
 M=0.343; 
 Iw=1/12*mw*R^2; 
 Ip=(1.9625e-05+3.66e-06)*2; 
 Im=0.0007838967;
 g=9.8;
% Perform physical calculations
 Nm=M*(x2+(L+Lm)*(theta2*cos(theta)-theta1^2*sin(theta))-l*(phi2*cos(phi)-phi1^2*sin(phi)));
 Pm=M*g+M*((L+Lm)*(-theta1^2*cos(theta)-theta2*sin(theta))-l*(phi1^2*cos(phi)+phi2*sin(phi)));
 N=Nm+mp*(x2+L*(theta2*cos(theta)-theta1^2*sin(theta)));
 P=Pm+mp*g+mp*L*(-theta1^2*cos(theta)-theta2*sin(theta));
 equ1=x2-(T-N*R)/(Iw/R+mw*R);
 equ2=(P*L+Pm*Lm)*sin(theta)-(N*L+Nm*Lm)*cos(theta)-T+Tp-Ip*theta2;
 equ3=Tp+Nm*l*cos(phi)+Pm*l*sin(phi)-Im*phi2;
 [x2,theta2,phi2]=solve(equ1,equ2,equ3,x2,theta2,phi2);
% Calculate Jacobian matrix, then obtain state-space equations
 Ja=jacobian([theta1;theta2;x1;x2;phi1;phi2],[theta theta1 x x1 phi phi1]);
 Jb=jacobian([theta1;theta2;x1;x2;phi1;phi2],[T Tp]);
 A=vpa(subs(Ja,[theta theta1 x x1 phi phi1],[0 0 0 0 0 0]));
 B=vpa(subs(Jb,[theta theta1 x x1 phi phi1],[0 0 0 0 0 0]));
% Discretization
 [G,H]=c2d(eval(A),eval(B),0.005);
% Define weight matrices Q, R
 Q=diag([1 10 100 20 1000 1]);
 R=diag([1 1]);
% Solve for feedback matrix K
 Ks(:,:,step)=dlqr(G,H,Q,R);
end

% 对K的每个元素关于L0进行拟合
K=sym('K',[2 6]);
syms L0;
for x=1:2
    for y=1:6
        p=polyfit(L0s,reshape(Ks(x,y,:),1,length(L0s)),3);
        K(x,y)=p(1)*L0^3+p(2)*L0^2+p(3)*L0+p(4);
    end
end

% 输出到m函数
matlabFunction(K,'File','myfile');

% 代入L0=0.07打印矩阵K
vpa(subs(K,L0,0.08))
