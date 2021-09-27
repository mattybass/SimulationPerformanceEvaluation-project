clear 
n=2;
d=1;
mu=0;
sigma=0.3;
Gr=1;
R=0.001;
f=2.4;
nprop = 2; 
%Gt=rho;
Pt=20;
%+normrnd(mu,sigma);
%RSSI=@(A,n,d,mu,sigma) A-10*log2(d);
RSSI=@(A,n,sigma,d) A-10*nprop*log2(d)+ normrnd(0,0.5);
%+normrnd(mu,sigma);
Pr=@(Pt,Gt,Gr,R,f) Pt+Gt+Gr-20*log10(R)-20*log2(f)-92.45;
%ComputeAngle=@(angle_temp) rho(angle_temp);
computeDist=@(A,RSSI,n) 2.^((A-RSSI)/(10*n));

x=0:0.01:10;
y=0:0.01:5;

coord_X=2;
coord_Y=3;
dist_X=-ones(size(y'))*(coord_X-x);
dist_Y=(coord_Y-y')*ones(size(x));
dist=dist_X.^2+dist_Y.^2;
mask=((dist_Y(:,1)>0*ones(size(x))) & (ones(size(y'))*dist_X(1,:)>0))+...
       2*((dist_Y(:,1)>0*ones(size(x))) & (ones(size(y'))*dist_X(1,:)<0))+...
        3*((dist_Y(:,1)<0*ones(size(x))) & (ones(size(y'))*dist_X(1,:)<0))+...
        4*((dist_Y(:,1)<0*ones(size(x))) & (ones(size(y'))*dist_X(1,:)>0));
angle=atan(dist_Y./dist_X);
angle_def=angle.*(mask==1)+ ((pi+angle).*(mask==2))+ ((pi+angle).*(mask==3))+ ((2*pi+angle).*(mask==4));
thickness=0.0001;
theta = 0:thickness:2*pi;
%rho=abs(cos(pi/2.*cos(theta-pi/2))./sin(theta-pi/2)).^2;
rho=ones(size(theta))*1.64;


%matrix_Pr=Pr(Pt,angle_index,Gr,R,f);
%matrix=RSSI(60,n,dist,mu,sigma);



computeDist=@(A,RSSI,n) 2^((A-RSSI)/(10*n));
sigma=0.2;


x=0:0.01:10;
y=0:0.01:5;

n=2;A=60;
%n_r=[2,2];
%<<<<<<< Updated upstream
n_r=[nprop];
%>>>>>>> Stashed changes
c_1=[3,4];c_2=[8,2];c_3=[9,3]; c_4 = [7,2] ; c_5 = [1,3.5];



figure();
dist_X=-ones(size(y'))*(c_1(1)-x);
dist_Y=(c_1(2)-y')*ones(size(x));
dist_1=sqrt(dist_X.^2+dist_Y.^2);
m_1=RSSI(60,n,sigma,dist_1);
imagesc(m_1);
xticks(0:100:1000);xticklabels([0:1:10]);yticks(0:100:500);yticklabels([0:1:5]);
clear dist_X dist_Y
%%
figure();
dist_X=-ones(size(y'))*(c_2(1)-x);
dist_Y=(c_2(2)-y')*ones(size(x));
dist_2=sqrt(dist_X.^2+dist_Y.^2);
m_2=RSSI(60,n,sigma,dist_2);

imagesc(m_2);
xticks(0:100:1000);xticklabels([0:1:10]);yticks(0:100:500);yticklabels([0:1:5]);
clear dist_X dist_Y

figure();
dist_X=-ones(size(y'))*(c_3(1)-x);
dist_Y=(c_3(2)-y')*ones(size(x));
dist_3=sqrt(dist_X.^2+dist_Y.^2);
m_3=RSSI(60,n,sigma,dist_3);

imagesc(m_3);
xticks(0:100:1000);xticklabels([0:1:10]);yticks(0:100:500);yticklabels([0:1:5]);
clear dist_X dist_Y


figure();
dist_X=-ones(size(y'))*(c_4(1)-x);
dist_Y=(c_4(2)-y')*ones(size(x));
dist_4=sqrt(dist_X.^2+dist_Y.^2);
m_4=RSSI(60,n,sigma,dist_4);

figure();
dist_X=-ones(size(y'))*(c_5(1)-x);
dist_Y=(c_5(2)-y')*ones(size(x));
dist_5=sqrt(dist_X.^2+dist_Y.^2);
m_5=RSSI(60,n,sigma,dist_5);


imagesc(m_4);
xticks(0:100:1000);xticklabels([0:1:10]);yticks(0:100:500);yticklabels([0:1:5]);
clear dist_X dist_Y



pos=[4,2];
iPos(1)=find(x==pos(1));
iPos(2)=find(y==pos(2));
RSSI_vect=m_1(iPos(2),iPos(1));m_dist=computeDist(A,RSSI_vect(1),n_r(1));
RSSI_vect(2)=m_2(iPos(2),iPos(1));m_dist(2)=computeDist(A,RSSI_vect(2),n_r(1));
RSSI_vect(3)=m_3(iPos(2),iPos(1));m_dist(3)=computeDist(A,RSSI_vect(3),n_r(1));
RSSI_vect(4)=m_4(iPos(2),iPos(1));m_dist(4)=computeDist(A,RSSI_vect(4),n_r(1));
RSSI_vect(5)=m_5(iPos(2),iPos(1));m_dist(5)=computeDist(A,RSSI_vect(5),n_r(1));


% maximum likelihood 
pos_x = [c_1(1),c_2(1),c_3(1),c_4(1),c_5(1)];
pos_y = [c_1(2),c_2(2),c_3(2),c_4(2),c_5(2)];
s = size(pos_x,2);

 
 
  for   i = 1: (s-1)
       A(i,1) = 2*(pos_x(i) - (pos_x(s))); 
       A(i,2)=   2*(pos_y(i) - (pos_y(s)));  
       b(i,1)= pos_x(i).^2 - pos_x(s).^2 + pos_y(i).^2 - pos_y(s).^2 + m_dist(s).^2-m_dist(i).^2; 
  end 
 
  
  position = inv(A' * A)* A' *b ;
  
  
 


