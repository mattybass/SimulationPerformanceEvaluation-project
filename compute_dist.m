function [d_real,d_model] = compute_dist(x,y,position_beacon,shadowing,fading,position_real,RSSI,computeDist)

n=2;
A=60;

iPos(1)=find(x==position_real(1));
iPos(2)=find(y==position_real(2));
s=size(position_beacon,1);

dist=cell(1,s);
m_model=cell(1,s);
m_real=cell(1,s);
RSSI_model=zeros(1,s);
RSSI_real=zeros(1,s);
d_model=zeros(1,s);
d_real=zeros(1,s);

for i=1:size(position_beacon,1)
        dist_X=-ones(size(y'))*(position_beacon(i,1)-x);
        dist_Y=(position_beacon(i,2)-y')*ones(size(x));
        dist{i}=sqrt(dist_X.^2+dist_Y.^2);
        m_model{i}=RSSI(60,n,dist{i});
        m_real{i}=m_model{i}+shadowing+fading{i};
        %figure();
        %imagesc(m{i});
        %xticks(0:100:1000);xticklabels([0:1:10]);yticks(0:100:500);yticklabels([0:1:5]);
        clear dist_X dist_Y 
        RSSI_model(i)=m_model{i}(iPos(2),iPos(1));d_model(i)=computeDist(A,RSSI_model(i),n(1));
        RSSI_real(i)=m_real{i}(iPos(2),iPos(1));d_real(i)=computeDist(A,RSSI_real(i),n(1));
        
    end
end

