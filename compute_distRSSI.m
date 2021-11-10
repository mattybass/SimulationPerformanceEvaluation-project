function [dist_RSSI] = compute_distRSSI(beacon,points,shadowing,fading,model,space)


%iPos(1)=find(x==position_real(1));
%iPos(2)=find(y==position_real(2));
%s=size(beacon.position,1);

%m_real=cell(1,s);
%RSSI_real=zeros(1,s);
%d_real=zeros(1,s);
noise=normrnd(model.noise.mu,model.noise.sigma,numel(space.x),numel(space.y),size(space.beacon_size,1));
RSSI_real=model.RSSI(model.A,model.n,beacon.dist)+shadowing+fading+noise;
%RSSI_real_1=RSSI_real(points.index_x,points.index_y,:);
%points.index_x,points.index_y,i
dist_RSSI=model.computeDist(model.A,RSSI_real(points.index_x,points.index_y,:),model.n);
%{
for i=1:size(position_beacon,1)
        m_real{i}=RSSI(60,n,dist{i})+shadowing+fading{i};
        %figure();
        %imagesc(m{i});
        %xticks(0:100:1000);xticklabels([0:1:10]);yticks(0:100:500);yticklabels([0:1:5]);
        clear dist_X dist_Y 
        RSSI_real(i)=m_real{i}(iPos(1),iPos(2));d_real(i)=computeDist(A,RSSI_real(i),n);
        
    end
%}
end

