clear 
n=2;
d=1;
mu=0;
sigma=0.3;
Gr=1;
R=0.001;
f=2.4;
n_r = 2; 
Pt=20;
RSSI=@(A,n,sigma,d) A-10*n*log2(d) + normrnd(0,0.5);
Pr=@(Pt,Gt,Gr,R,f) Pt+Gt+Gr-20*log10(R)-20*log2(f)-92.45;

computeDist=@(A,RSSI,n) 2.^((A-RSSI)/(10*n));

x=0:0.01:10;
y=0:0.01:5;
%{
coord_X=2;
coord_Y=3;
dist_X=-ones(size(y'))*(coord_X-x);
dist_Y=(coord_Y-y')*ones(size(x));
dist=dist_X.^2+dist_Y.^2;
%}
computeDist=@(A,RSSI,n) 2^((A-RSSI)/(10*n));
sigma=0.2;

n=2;A=60;
beacon_pos=[3,4; 8,2; 9,3; 7,2; 1,3.5];
%c_1=[3,4];c_2=[8,2];c_3=[9,3]; c_4 = [7,2] ; c_5 = [1,3.5];
position_wanted=[4,2];
iPos(1)=find(x==position_wanted(1));
iPos(2)=find(y==position_wanted(2));

for i=1:size(beacon_pos,1)
   %figure();
    dist_X=-ones(size(y'))*(beacon_pos(i,1)-x);
    dist_Y=(beacon_pos(i,2)-y')*ones(size(x));
    dist{i}=sqrt(dist_X.^2+dist_Y.^2);
    m{i}=RSSI(60,n,sigma,dist{i});
    %imagesc(m{i});
    %xticks(0:100:1000);xticklabels([0:1:10]);yticks(0:100:500);yticklabels([0:1:5]);
    clear dist_X dist_Y 
    RSSI_vect(i)=m{i}(iPos(2),iPos(1));d(i)=computeDist(A,RSSI_vect(i),n_r(1));
end

% maximum likelihood 
pos_x = beacon_pos(:,1);
pos_y = beacon_pos(:,2);
s = size(pos_x,1);

for i=1:(s-1)
       A(i,1) = 2*(pos_x(i) - (pos_x(s))); 
       A(i,2) = 2*(pos_y(i) - (pos_y(s)));  
       b(i,1) = pos_x(i).^2 - pos_x(s).^2 + pos_y(i).^2 - pos_y(s).^2 + d(s).^2-d(i).^2; 
end

position_LSM = inv(A' * A)* A' * b ;
iPos(1)=find(x==position_wanted(1));
iPos(2)=find(y==position_wanted(2));
%% Uniform sampling
beta=0.5;
thick=1000;
PSO_x=linspace(position_LSM(1)-beta,position_LSM(1)+beta,thick);
PSO_y=linspace(position_LSM(2)-beta,position_LSM(2)+beta,thick);
f_mat=zeros(size(PSO_x,2),size(PSO_y,2));
for k=1:s
    for i=1:size(PSO_x,2)
        for j=1:size(PSO_y,2)
            f_mat(i,j)=f_mat(i,j)+(d(k)-norm([PSO_x(i),PSO_y(j)]-beacon_pos(k,:)))^2;
        end
    end
end


[x_index,y_index]=find(f_mat==min(f_mat,[],'all'));
position_OPT=[position_LSM(1)-beta+2*beta/thick*x_index;position_LSM(2)-beta+2*beta/thick*y_index];

%% PSO
f_i=@(r_pos,b_pos,d) (d-norm(r_pos-b_pos))^2;

fitness = @(x) f_i(x,beacon_pos(1,:),d(1))+ ...
                 f_i(x,beacon_pos(2,:),d(2))+ ...
                 f_i(x,beacon_pos(3,:),d(3))+ ...
                 f_i(x,beacon_pos(4,:),d(4))+ ...
                 f_i(x,beacon_pos(5,:),d(5));
            
%(d(k)-norm([PSO_x(i),PSO_y(j)]-beacon_pos(k,:)))^2;

rng default  % For reproducibility
nvars = 2;
pos_PSO = particleswarm(fitness,nvars);
