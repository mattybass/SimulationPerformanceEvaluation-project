clear 
n=2;
d=1;
mu=0;
sigma=2;
Gr=1;
R=0.001;
f=2.4;
%Gt=rho;
Pt=20;
%+normrnd(mu,sigma);
RSSI=@(A,n,d,mu,sigma) A-10*normrnd(n,0.5)*log10(d);
Pr=@(Pt,Gt,Gr,R,f) Pt+Gt+Gr-20*log10(R)-20*log2(f)-92.45;
ComputeAngle=@(angle_temp) rho(angle_temp);
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
%%
for i=1:size(angle_def,1)
    for j=1:size(angle_def,2)
        %{   
        a=angle_def(i,j);
            tol = 5 * eps(a); % A very small value.
            angle_index(i,j)=tetha(tetha>a-tol && tetha<a+tol);
        %}
        if(~isnan(angle_def(i,j)))
            angle_index(i,j)=rho(round(angle_def(i,j)/thickness+1));
        else
            angle_index(i,j)=NaN;
        end
    end
end
%%
%gain=rho(find(theta,angle_def));
%P=Pr(Pt,Gt,Gr,R,f);
angle_index=NaN(size(angle_def));
angle_temp=round((angle_def.*~isnan(angle_def))/thickness+1);
for i=1:size(angle_temp,1)
    angle_index(i,:)=rho(angle_temp(i,:)~=nan);
    %angle_index(i,:)=rho(angle_temp(~isnan(angle_temp(i,:))));
end
imagesc(angle_index);
%%
matrix=RSSI(60,n,dist,mu,sigma);
imagesc(matrix);
colorbar;
theta = 0:0.0001:2*pi;
rho=abs(cos(pi/2.*cos(theta-pi/2))./sin(theta-pi/2)).^2*1.64;



polarplot(theta,10* log10(rho));
polarplot(theta,ones(size(theta)));

%friss


%polarplot(theta,Pr);

%A=Pr(Pt,angle_index,Gr,R,f);

%%
mask_dist=dist<1.01&dist>0.99;
disc=0:pi/10:2*pi;
thick_2=0:pi/10:2*pi;
mask_angle=discretize(angle_def,0:pi/10:2*pi)*pi/10;
mask=mask_dist.*mask_angle;
only_angle=mask(mask~=0);



for i=2:numel(disc)
    tol = 5 * eps(disc(i));
    m(i)=mean(angle_index((mask<disc(i)+tol & mask>disc(i)-tol) & mask_dist==1));
end

%%
matrix_Pr=Pr(Pt,angle_index,Gr,R,f);
matrix=RSSI(60,n,dist,mu,sigma);

imagesc(matrix);
%% prova
computeDist=@(A,RSSI,n) 2^((A-RSSI)/(10*n));
sigma=0.2;
nprop = normrnd(n,sigma);
RSSI=@(A,n,sigma,d) A-(10*nprop*log2(d)); %normrnd(n,0.5)
x=0:0.01:10;
y=0:0.01:5;
<<<<<<< HEAD
sigma=0.1;
n=2;A=60;
%n_r=[2,2];
<<<<<<< HEAD
n_r=[1.9,2.1];
=======
<<<<<<< Updated upstream
=======

n=2;A=60;
%n_r=[2,2];
%<<<<<<< Updated upstream
>>>>>>> main
n_r=[1.4,n];
%=======
n_r=[1.4,nprop];
<<<<<<< HEAD
>>>>>>> Stashed changes
>>>>>>> main
=======
%>>>>>>> Stashed changes
>>>>>>> main
c_1=[1,1];c_2=[1,4.5];c_3=[9,3];
pos=[4,3];


figure();
dist_X=-ones(size(y'))*(c_1(1)-x);
dist_Y=(c_1(2)-y')*ones(size(x));
dist_1=sqrt(dist_X.^2+dist_Y.^2);
m_1=RSSI(60,n,sigma,dist_1);
imagesc(m_1);
xticks(0:100:1000);xticklabels([0:1:10]);yticks(0:100:500);yticklabels([0:1:5]);
clear dist_X dist_Y

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


iPos(1)=find(x==pos(1));
iPos(2)=find(y==pos(2));
<<<<<<< HEAD
<<<<<<< HEAD
RSSI_1=m_1(iPos(2),iPos(1));d11=computeDist(A,RSSI_1,n_r(1));d12=computeDist(A,RSSI_1,n_r(2));
RSSI_2=m_2(iPos(2),iPos(1));d21=computeDist(A,RSSI_2,n_r(1));d22=computeDist(A,RSSI_2,n_r(2));
RSSI_3=m_3(iPos(2),iPos(1));d31=computeDist(A,RSSI_3,n_r(1));d32=computeDist(A,RSSI_3,n_r(2));
%%
=======
RSSI_1=m_1(iPos(2),iPos(1));d11=computeDist(A,RSSI_1,n_r(2));d12=computeDist(A,RSSI_1,n_r(1));
RSSI_2=m_2(iPos(2),iPos(1));d21=computeDist(A,RSSI_2,n_r(2));d22=computeDist(A,RSSI_2,n_r(1));
RSSI_3=m_3(iPos(2),iPos(1));d31=computeDist(A,RSSI_3,n_r(2));d32=computeDist(A,RSSI_3,n_r(1));
=======
RSSI_1=m_1(iPos(2),iPos(1));d11=computeDist(A,RSSI_1,n_r(1));d12=computeDist(A,RSSI_1,n_r(2));
RSSI_2=m_2(iPos(2),iPos(1));d21=computeDist(A,RSSI_2,n_r(1));d22=computeDist(A,RSSI_2,n_r(2));
RSSI_3=m_3(iPos(2),iPos(1));d31=computeDist(A,RSSI_3,n_r(1));d32=computeDist(A,RSSI_3,n_r(2));
>>>>>>> main

%<<<<<<< Updated upstream\
% Intersezione c_1 con c_2
>>>>>>> main
dist_prova=sqrt((pos(1)-c_3(1)).^2+(pos(2)-c_3(2)).^2);
hold on;
%<<<<<<< Updated upstream
[x12_1,y12_1] = circcirc(c_1(2),c_1(1),d11,c_2(2),c_2(1),d22);
[x12_2,y12_2] = circcirc(c_1(2),c_1(1),d11,c_3(2),c_3(1),d32);
[x13_1,y13_1] = circcirc(c_3(2),c_3(1),d31,c_2(2),c_2(1),d22);
[x13_2,y13_2] = circcirc(c_3(2),c_3(1),d31,c_1(2),c_1(1),d12);
[x23_1,y23_1] = circcirc(c_2(2),c_2(1),d21,c_1(2),c_1(1),d12);
[x23_2,y23_2] = circcirc(c_2(2),c_2(1),d21,c_3(2),c_3(1),d32);


pos_new=[4,2];
iPos_new(1)=find(x==pos_new(1));
iPos_new(2)=find(y==pos_new(2));
m_3(iPos_new(2),iPos_new(1))=1000;
imagesc(m_3);
%{
r_1(1)=find(x==c_1(1));r_1(2)=find(y==c_1(2));
r_2(1)=find(x==c_2(1));r_2(2)=find(y==c_2(2));
r_3(1)=find(x==c_3(1));r_3(2)=find(y==c_3(2));
new_matrix=ones(size(m_1));
new_matrix(r_1(2),r_1(1))=0;
new_matrix(r_2(2),r_2(1))=0;
new_matrix(r_3(2),r_3(1))=0;
new_matrix(iPos(2),iPos(1))=0;
new_matrix(iPos_new(2),iPos_new(1))=0;
imagesc(new_matrix);
colormap(bone);
xticks(0:100:1000);xticklabels([0:1:10]);yticks(0:100:500);yticklabels([0:1:5]);
%}
%%
theta = 0 : 0.01 : 2*pi;

figure();
scatter(c_1(1),c_1(2),'r');hold on;scatter(c_2(1),c_2(2),'b');hold on;scatter(c_3(1),c_3(2),'g');hold on;scatter(pos(1),pos(2),'x');hold on;
xlim([0,20]);ylim([0 10]);

x_1 = d12 * cos(theta) + c_1(1);y_1 = d12 * sin(theta) + c_1(2);
plot(x_1, y_1, 'r', 'LineWidth', 1);
hold on;
x_1 = d11 * cos(theta) + c_1(1);y_1 = d11 * sin(theta) + c_1(2);
plot(x_1, y_1, 'r', 'LineWidth', 1);
hold on;
x_1 = d21 * cos(theta) + c_2(1);y_1 = d21 * sin(theta) + c_2(2);
plot(x_1, y_1, 'b', 'LineWidth', 1);
hold on;
x_1 = d22 * cos(theta) + c_2(1);y_1 = d22 * sin(theta) + c_2(2);
plot(x_1, y_1, 'b', 'LineWidth', 1);
hold on;
x_1 = d31 * cos(theta) + c_3(1);y_1 = d31 * sin(theta) + c_3(2);
plot(x_1, y_1, 'g', 'LineWidth', 1);
hold on;
x_1 = d32 * cos(theta) + c_3(1);y_1 = d32 * sin(theta) + c_3(2);
plot(x_1, y_1, 'g', 'LineWidth', 1);
hold on;

x_2 = d21 * cos(theta) + c_2(1);y_2 = d21 * sin(theta) + c_2(2);
plot(x_2, y_2, 'r-', 'LineWidth', 1);
hold on;
x_2 = d22 * cos(theta) + c_2(1);y_2 = d22 * sin(theta) + c_2(2);
plot(x_2, y_2, 'r-', 'LineWidth', 1);

x_3 = d31 * cos(theta) + c_3(1);y_3 = d31 * sin(theta) + c_3(2);
plot(x_3, y_3, 'r-', 'LineWidth', 1);
hold on;
x_3 = d32 * cos(theta) + c_3(1);y_3 = d32 * sin(theta) + c_3(2);
plot(x_3, y_3, 'r-', 'LineWidth', 1);

hold on;
eps = 0.01; 
 ind_x_1 = find(x_2 - x_1 < eps);
 ind_y_1 = find(y_2 - y_1<eps);
 px = x_1(ind_x_1);
 py = y_1(ind_y_1);
 
 figure
plot(x_1, y_1, x_2, y_2, px, py, 'ro', 'MarkerSize', 18)
axis([0 10 0 10]);
%xticklabels([0:1:10]);
%yticklabels([5:1:0]);

%{
=======
[xout_1,yout_1] = circcirc(c_1(1),c_1(2),d11,c_2(1),c_2(2),d22);
[xout_2,yout_2] = circcirc(c_1(1),c_1(2),d11,c_3(1),c_3(2),d32);
[xout_3,yout_3] = circcirc(c_3(1),c_3(2),d31,c_2(1),c_2(2),d22);
[xout_4,yout_4] = circcirc(c_3(1),c_3(2),d31,c_1(1),c_1(2),d12);
[xout_5,yout_5] = circcirc(c_2(1),c_2(2),d21,c_1(1),c_1(2),d12);
[xout_6,yout_6] = circcirc(c_2(1),c_2(2),d21,c_3(1),c_3(2),d32);

xCenter = c_1(2);
yCenter = c_1(1);
theta = 0 : 0.01 : 2*pi;
radius = d12;
x = radius * cos(theta) + xCenter;
y = radius * sin(theta) + yCenter;
imagesc(m_3);hold on;
plot(x, y, 'r-', 'LineWidth', 3);



dist_prova=sqrt((pos(1)-c_3(1)).^2+(pos(2)-c_3(2)).^2);

% Intersezione c_1 con c_3

[xout,yout] = circcirc(c_1(1),c_1(2),d12,c_3(1),c_3(2),d22);
>>>>>>> Stashed changes
xCenter = c_1(2);
yCenter = c_1(1);
theta = 0 : 0.01 : 2*pi;
radius = d12;
x = radius * cos(theta) + xCenter;
y = radius * sin(theta) + yCenter;
imagesc(m_3);hold on;
plot(x, y, 'r-', 'LineWidth', 3);
%}

%>>>>>>> Stashed changes

