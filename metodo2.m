clear 

RSSI=@(A,n,d) A-10*n*log10(d); 
computeDist=@(A,RSSI,n) 10.^((A-RSSI)/(10*n));

x=0:0.01:20;
y=0:0.01:10;

n=2;sigma=2;A=60;
beacon_pos=[3,4; 8,2; 9,3; 7,2; 1, 3.5];
position_real=[10,5];

chess_lenght=100;
shadowing=compute_shadowing(x,y,chess_lenght);

ntimes=20;

position_LSM=zeros(ntimes,2);
position_PSO=zeros(ntimes,2);
position_OPT=zeros(ntimes,2);

for i=1:ntimes
    fading=compute_fading(x,y,chess_lenght,beacon_pos);

    [d,d_model]=compute_dist(x,y,beacon_pos,shadowing,fading,position_real,RSSI,computeDist);
    position_LSM(i,:) = leastSquaresMethod(beacon_pos,d);
    position_PSO(i,:)=particleSwarmOptimizer(beacon_pos,d,position_LSM(i,:),0);
    position_OPT(i,:)=optimization_grid(beacon_pos,d,position_LSM(i,:));
    clear fading 
end

%% PSO

position_LSM_mean=[mean(position_LSM(:,1)),mean(position_LSM(:,2))];
position_PSO_mean=[mean(position_PSO(:,1)),mean(position_PSO(:,2))];
%position_OPT_mean=[mean(position_OPT(:,1)),mean(position_OPT(:,2))];
(sqrt( (position_LSM(:,1)-position_real(1)).^2+(position_LSM(:,2)-position_real(2)).^2 ))/size(position_LSM,1);
position_LSM_errors=sqrt( (position_LSM(:,1)-position_real(1)).^2+(position_LSM(:,2)-position_real(2)).^2 );
position_PSO_errors=sqrt( (position_PSO(:,1)-position_real(1)).^2+(position_PSO(:,2)-position_real(2)).^2 );
figure();
histogram(position_PSO_errors,200);
figure();
histogram(position_LSM_errors,200);
