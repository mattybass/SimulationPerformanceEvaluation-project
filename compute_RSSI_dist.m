function [d_real] = compute_RSSI_dist(x,y,position_beacon,dist,shadowing,fading,position_real,RSSI,computeDist)

n=2;
A=60;

%iPos(1)=find(x==position_real(1));
%iPos(2)=find(y==position_real(2));
s=size(position_beacon,1);

m_real=cell(1,s);
RSSI_real=zeros(1,s);
d_real=zeros(1,s);

for i=1:size(position_beacon,1)
        m_real{i}=RSSI(60,n,dist{i})+shadowing+fading{i};
        %figure();
        %imagesc(m{i});
        %xticks(0:100:1000);xticklabels([0:1:10]);yticks(0:100:500);yticklabels([0:1:5]);
        clear dist_X dist_Y 
        RSSI_real(i)=m_real{i}(iPos(1),iPos(2));d_real(i)=computeDist(A,RSSI_real(i),n(1));
        
    end
end

