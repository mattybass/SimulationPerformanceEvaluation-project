function [fading] = compute_fading(x,y,chess_lenght,beacon_pos)
    x_index=[1:chess_lenght:numel(x)];
    x_index(end+1)=numel(x);
    x_index=unique(x_index);
    
    y_index=[1:chess_lenght:numel(y)];
    y_index(end+1)=numel(y);
    y_index=unique(y_index);

    fading=cell(1,size(beacon_pos,1));
    for k=1:numel(fading)
        fading{k}=zeros(numel(y),numel(x));
        for i=1:size(x_index,2)-1
            for j=1:size(y_index,2)-1
                fading{k}(y_index(j):y_index(j+1),x_index(i):x_index(i+1))=binornd(1,0.50)*exprnd(1);
            end
        end
    end
end

