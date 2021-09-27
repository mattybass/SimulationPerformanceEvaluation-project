clear 
n=2;
d=1;
mu=0;
sigma=0.3;
Gr=1;
R=0.001;
f=2.4;
%Gt=rho;
Pt=20;
%+normrnd(mu,sigma);
RSSI_reale = @(A,n,d,mu,sigma) A-10*log2(d);
RSSI=@(A,n,d,mu,sigma) A-10*log2(d)+normrnd(mu,sigma);
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
matrix=RSSI(60,n,dist,mu,sigma);



computeDist=@(A,RSSI,n) 2^((A-RSSI)/(10*n));
sigma=0.2;
nprop = 2; 
RSSI=@(A,n,sigma,d) A-(10*nprop*log2(d)+normrnd(0,sigma)); %normrnd(n,0.5)
x=0:0.01:10;
y=0:0.01:5;

n=2;A=60;
%n_r=[2,2];
%<<<<<<< Updated upstream
n_r=[nprop];
%=======
n_r=[nprop];
%>>>>>>> Stashed changes
c_1=[3,4];c_2=[8,2];c_3=[9,3]; c_4 = [7,2] ; c_5 = [1,3.5];



figure();
dist_X=-ones(size(y'))*(c_1(1)-x);
dist_Y=(c_1(2)-y')*ones(size(x));
dist_1=sqrt(dist_X.^2+dist_Y.^2);
m_1=RSSI(60,n,sigma,dist_1);
m_1_Real = RSSI_reale(A,n,dist_1,mu,sigma);
imagesc(m_1);
xticks(0:100:1000);xticklabels([0:1:10]);yticks(0:100:500);yticklabels([0:1:5]);
clear dist_X dist_Y

figure();
dist_X=-ones(size(y'))*(c_2(1)-x);
dist_Y=(c_2(2)-y')*ones(size(x));
dist_2=sqrt(dist_X.^2+dist_Y.^2);
m_2=RSSI(60,n,sigma,dist_2);
m_2_Real = RSSI_reale(A,n,dist_2,mu,sigma);

imagesc(m_2);
xticks(0:100:1000);xticklabels([0:1:10]);yticks(0:100:500);yticklabels([0:1:5]);
clear dist_X dist_Y

figure();
dist_X=-ones(size(y'))*(c_3(1)-x);
dist_Y=(c_3(2)-y')*ones(size(x));
dist_3=sqrt(dist_X.^2+dist_Y.^2);
m_3=RSSI(60,n,sigma,dist_3);
m_3_Real = RSSI_reale(A,n,dist_3,mu,sigma);

imagesc(m_3);
xticks(0:100:1000);xticklabels([0:1:10]);yticks(0:100:500);yticklabels([0:1:5]);
clear dist_X dist_Y


figure();
dist_X=-ones(size(y'))*(c_4(1)-x);
dist_Y=(c_4(2)-y')*ones(size(x));
dist_4=sqrt(dist_X.^2+dist_Y.^2);
m_4=RSSI(60,n,sigma,dist_4);
m_4_Real = RSSI_reale(A,n,dist_4,mu,sigma);

figure();
dist_X=-ones(size(y'))*(c_5(1)-x);
dist_Y=(c_5(2)-y')*ones(size(x));
dist_5=sqrt(dist_X.^2+dist_Y.^2);
m_5=RSSI(60,n,sigma,dist_5);
m_5_Real = RSSI_reale(A,n,dist_5,mu,sigma);


imagesc(m_4);
xticks(0:100:1000);xticklabels([0:1:10]);yticks(0:100:500);yticklabels([0:1:5]);
clear dist_X dist_Y



pos=[4,2];
iPos(1)=find(x==pos(1));
iPos(2)=find(y==pos(2));
RSSI_1=m_1(iPos(2),iPos(1));d11=computeDist(A,RSSI_1,n_r(1));
RSSI_2=m_2(iPos(2),iPos(1));d21=computeDist(A,RSSI_2,n_r(1));
RSSI_3=m_3(iPos(2),iPos(1));d31=computeDist(A,RSSI_3,n_r(1));
RSSI_4=m_4(iPos(2),iPos(1));d41=computeDist(A,RSSI_4,n_r(1));
RSSI_5=m_5(iPos(2),iPos(1));d51=computeDist(A,RSSI_5,n_r(1));
%distanze di riferimento (reali)
RSSI_1_real =m_1_Real(iPos(2),iPos(1));d11_real=computeDist(A,RSSI_1_real,2);
RSSI_2_real =m_2_Real(iPos(2),iPos(1));d12_real=computeDist(A,RSSI_2_real,2);
RSSI_3_real =m_3_Real(iPos(2),iPos(1));d13_real=computeDist(A,RSSI_3_real,2);
RSSI_4_real =m_4_Real(iPos(2),iPos(1));d14_real=computeDist(A,RSSI_4_real,2);
RSSI_5_real =m_5_Real(iPos(2),iPos(1));d15_real=computeDist(A,RSSI_5_real,2);







% maximum likelihood 
positions_matrix_x = [c_1(1),c_2(1),c_3(1),c_4(1),c_5(1)];
positions_matrix_y = [c_1(2),c_2(2),c_3(2),c_4(2),c_5(2)];
distances_matrix = [d11_real,d12_real,d13_real,d14_real,d15_real];
s = size(positions_matrix_x,2);

 
 
  for   i = 1: (s-1)
       matrixA(i,1) = 2*(positions_matrix_x(i) - (positions_matrix_x(1,s))); 
       i = i+1 ; 
  end 
  
  for j = 1:(s-1)
     matrixA(j,2)=   2*(positions_matrix_y(j) - (positions_matrix_y(1,s))); 
       j = j+1 ; 
  end

  
 
  for i = 1:(s-1)
      matrixB(i,1)= ((positions_matrix_x(i).^2) - (positions_matrix_x(1,s).^2))+ ((positions_matrix_y(i).^2) - (positions_matrix_y(1,s).^2)) + (((d15_real).^2)-((distances_matrix(i).^2)))  ; 
      
      i = i+1 ;  
      

  end
  
  position = inv(matrixA'* matrixA)*(matrixA')*(matrixB) ;
  
  
 


