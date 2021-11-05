function [out] = print_graph(graph)
    out=figure();    
    imagesc(graph');
    xlabel("x");
    ylabel("y");
end

