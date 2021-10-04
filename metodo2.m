clear 
mu=0;sigma=0.5;
Gr=1;
R=0.001;
f=2.4;
Pt=20;
RSSI=@(A,n,sigma,d) A-10*n*log2(d) + normrnd(mu,sigma);

%raylrnd(B)

Pr=@(Pt,Gt,Gr,R,f) Pt+Gt+Gr-20*log10(R)-20*log2(f)-92.45;

computeDist=@(A,RSSI,n) 2.^((A-RSSI)/(10*n));

x=0:0.01:10;
y=0:0.01:5;

n=2;A=60;
beacon_pos=[3,4; 8,2; 9,3; 7,2; 1, 3.5];
position_real=[4,2];
iPos(1)=find(x==position_real(1));
iPos(2)=find(y==position_real(2));

for i=1:size(beacon_pos,1)
   %figure();
    dist_X=-ones(size(y'))*(beacon_pos(i,1)-x);
    dist_Y=(beacon_pos(i,2)-y')*ones(size(x));
    dist{i}=sqrt(dist_X.^2+dist_Y.^2);
    m{i}=RSSI(60,n,sigma,dist{i});
    %imagesc(m{i});
    %xticks(0:100:1000);xticklabels([0:1:10]);yticks(0:100:500);yticklabels([0:1:5]);
    clear dist_X dist_Y 
    RSSI_vect(i)=m{i}(iPos(2),iPos(1));d(i)=computeDist(A,RSSI_vect(i),n(1));
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
iPos(1)=find(x==position_real(1));
iPos(2)=find(y==position_real(2));
%% Uniform sampling
%{
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
%}
%% PSO

beta=0.5;
vect_fit=cell(1,3);
for i=1:s
    vect_fit{i}{1}=beacon_pos(i,:);
    vect_fit{i}{2}=d(i);
end

fitness=@(pos) sum( cellfun( @(prova) ((prova{2}-norm(pos-prova{1}))^2) ,vect_fit));

rng default  % For reproducibility
nvars = 2;
position_PSO = particleswarm(fitness,nvars,[position_LSM(1)-beta,position_LSM(2)-beta],[position_LSM(1)+beta,position_LSM(2)+beta]);
