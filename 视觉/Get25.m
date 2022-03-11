function arg = Get25(image)
    width = size(image,2);
    height = size(image,1);
    col_edge(1) = 1;
    col_edge(2) = ceil(width/5);
    col_edge(3) = 2*col_edge(2);
    col_edge(4) = 3*col_edge(2);
    col_edge(5) = 4*col_edge(2);
    col_edge(6) = ceil(width);
    row_edge(1) = 1;
    row_edge(2) = ceil(height/5);
    row_edge(3) = 2*row_edge(2);
    row_edge(4) = 3*row_edge(2);
    row_edge(5) = 4*row_edge(2);
    row_edge(6) = ceil(height);
    
    white_cnt = zeros(1,25);
    black_cnt = zeros(1,25);
    for i = 1:height
        for j = 1:width
            white_cnt(5*ceil(i/row_edge(2)) + ceil(j/col_edge(2)) -5) = (image(i,j)==1) + white_cnt(5*ceil(i/row_edge(2)) + ceil(j/col_edge(2)) -5);
            black_cnt(5*ceil(i/row_edge(2)) + ceil(j/col_edge(2)) -5) = (image(i,j)==0) + black_cnt(5*ceil(i/row_edge(2)) + ceil(j/col_edge(2)) -5);
        end
    end
    for i = 1:25
       if (white_cnt(i)+black_cnt(i)) == 0
           arg(i)=0;
       else
           arg(i) = white_cnt(i)/(white_cnt(i)+black_cnt(i));
       end
    end
end


