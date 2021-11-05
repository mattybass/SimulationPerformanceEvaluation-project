clear 

RSSI=@(A,n,d) A-10*n*log10(d); 
computeDist=@(A,RSSI,n) 10.^((A-RSSI)/(10*n));

x=0:0.01:20;
y=0:0.01:10;

n=2;sigma=2;A=-60;
beacon.position=[3,4; 8,2; 9,3; 7,2; 1, 3.5];
position_real=[10,5];
position=1;
chess_lenght=100;

%stop_times=[100:100:ntimes];
%{
position_LSM=zeros(ntimes,2);
position_PSO=zeros(ntimes,2);
position_OPT=zeros(ntimes,2);
%}
for i=1:size(beacon.position,1)
    dist_X=(beacon.position(i,1)-x)'*ones(size(y));
    dist_Y=ones(size(x))'*(beacon.position(i,2)-y);
    beacon.dist{i}=sqrt(dist_X.^2+dist_Y.^2);
    temp_x=mod(x,0.2);
    temp_y=mod(y,0.2);
    index_x=find(temp_x==0); %point to take RSSI
    index_y=find(temp_y==0);
    beacon.sampled_dist(:,:,i)=beacon.dist{i}(index_x,index_y);% z with array of distances
    
    clear dist_X dist_Y temp_x temp_y
end
shadowing=compute_shadowing(x,y,chess_lenght);

ntimes=5;
for i=1:ntimes
    [fading,fading_mean]=compute_fading(x,y,chess_lenght,beacon.position);
    dist_RSSI=compute_RSSI_dist(x,y,beacon.position,beacon.dist,shadowing,fading,position_real,RSSI,computeDist);    
end

    position_LSM(i,:) = leastSquaresMethod(beacon.position,dist_RSSI);
    position_PSO(i,:)=particleSwarmOptimizer(beacon.position,dist_RSSI,position_LSM(i,:),0);
    position_OPT(i,:)=optimization_grid(beacon.position,dist_RSSI,position_LSM(i,:));
    clear fading
    if(sum(stop_times==i)~=0)
        position_LSM_mean(find(stop_times==i),:)=[mean(position_LSM(:,1)),mean(position_LSM(:,2))];
        position_PSO_mean(find(stop_times==i),:)=[mean(position_PSO(:,1)),mean(position_PSO(:,2))];
        errors(find(stop_times==i),1)=(sum(sqrt( (position_LSM(:,1)-position_real(1)).^2+(position_LSM(:,2)-position_real(2)).^2 )))/size(position_LSM,1);
        errors(find(stop_times==i),2)=(sum(sqrt( (position_PSO(:,1)-position_real(1)).^2+(position_PSO(:,2)-position_real(2)).^2 )))/size(position_PSO,1);
        %position_LSM_errors=sqrt( (position_LSM(:,1)-position_real(1)).^2+(position_LSM(:,2)-position_real(2)).^2 );
        %position_PSO_errors=sqrt( (position_PSO(:,1)-position_real(1)).^2+(position_PSO(:,2)-position_real(2)).^2 );
    end
    
    %{
    %temp=[position_LSM(:,1);position_PSO(:,1)];
    %CI_x=[mean(temp)-1.96*std(temp)/mean(temp),mean(temp)+1.96*std(temp)/sqrt(numel(temp))];
    %temp=[position_LSM(:,2);position_PSO(:,2)];
    %CI_y=[mean(temp)-1.96*std(temp)/mean(temp),mean(temp)+1.96*std(temp)/sqrt(numel(temp))];  
    eta=2.58;
    CI_LSM_x=[mean(position_LSM(:,1))-1.96*std(position_LSM(:,1))/sqrt(numel(position_LSM(:,1))),mean(position_LSM(:,1))+eta*std(position_LSM(:,1))/sqrt(numel(position_LSM(:,1)))];
    CI_LSM_y=[mean(position_LSM(:,2))-1.96*std(position_LSM(:,2))/sqrt(numel(position_LSM(:,2))),mean(position_LSM(:,2))+eta*std(position_LSM(:,2))/sqrt(numel(position_LSM(:,2)))];
    CI_PSO_x=[mean(position_PSO(:,1))-1.96*std(position_PSO(:,1))/sqrt(numel(position_PSO(:,1))),mean(position_PSO(:,1))+eta*std(position_PSO(:,1))/sqrt(numel(position_PSO(:,1)))];
    CI_PSO_y=[mean(position_PSO(:,2))-1.96*std(position_PSO(:,2))/sqrt(numel(position_PSO(:,2))),mean(position_PSO(:,2))+eta*std(position_PSO(:,2))/sqrt(numel(position_PSO(:,2)))];
    %prova_LSM=position_LSM(position_LSM(:,1)>CI_x(1)&position_LSM(:,1)<CI_x(2)&position_LSM(:,2)>CI_y(1)&position_LSM(:,2)<CI_y(2),:);
    %prova_PSO=position_PSO(position_PSO(:,1)>CI_x(1)&position_PSO(:,1)<CI_x(2)&position_PSO(:,2)>CI_y(1)&position_PSO(:,2)<CI_y(2),:);
    
    save_LSM=position_LSM(position_LSM(:,1)>CI_LSM_x(1)&position_LSM(:,1)<CI_LSM_x(2)&position_LSM(:,2)>CI_LSM_y(1)&position_LSM(:,2)<CI_LSM_y(2),:);
    save_PSO=position_PSO(position_PSO(:,1)>CI_PSO_x(1)&position_PSO(:,1)<CI_PSO_x(2)&position_PSO(:,2)>CI_PSO_y(1)&position_PSO(:,2)<CI_PSO_y(2),:);   
    [dist_RSSI]=compute_dist(x,y,beacon_pos,dist,shadowing,fading_mean,position_real,RSSI,computeDist);

    position_LSM_asympt = leastSquaresMethod(beacon_pos,dist_RSSI);
    position_PSO_asympt= particleSwarmOptimizer(beacon_pos,dist_RSSI,position_LSM(i,:),0);
    prova= [position_OPT,position_PSO];
    %}
%% PSO

position_LSM_mean=[mean(position_LSM(:,1)),mean(position_LSM(:,2))];
position_PSO_mean=[mean(position_PSO(:,1)),mean(position_PSO(:,2))];
%{
%position_OPT_mean=[mean(position_OPT(:,1)),mean(position_OPT(:,2))];
(sqrt( (position_LSM(:,1)-position_real(1)).^2+(position_LSM(:,2)-position_real(2)).^2 ))/size(position_LSM,1);
position_LSM_errors=sqrt( (position_LSM(:,1)-position_real(1)).^2+(position_LSM(:,2)-position_real(2)).^2 );
position_PSO_errors=sqrt( (position_PSO(:,1)-position_real(1)).^2+(position_PSO(:,2)-position_real(2)).^2 );
%}
%figure();
%histogram(position_PSO_errors,200);
%figure();
%histogram(position_LSM_errors,200);

%prova_LSM=position_LSM(position_LSM(:,1)>CI_x(1)&position_LSM(:,1)<CI_x(2)&position_LSM(:,2)>CI_y(1)&position_LSM(:,2)<CI_y(2),:);
%prova_PSO=position_PSO(position_PSO(:,1)>CI_x(1)&position_PSO(:,1)<CI_x(2)&position_PSO(:,2)>CI_y(1)&position_PSO(:,2)<CI_y(2),:);
%%
k=3;
choice=kmeans(position_LSM,k);
colors={'g','b','r','c','y'};
figure()
for i=1:k
    scatter(position_LSM(choice==i,1),position_LSM(choice==i,2),colors{i});
    hold on;
end