function [value] = Kmeans(image)
    image = double(image);
    KMEANS_K = 4;
    m = [51,0,0;102,0,0;153,0,0;206,0,0];%第一列存最终分类结果，第二列存每种分类的样本数，第三列存中间变量
    maxtimes = 2;
    for time = 1:maxtimes
        for i = 1:KMEANS_K
            m(i,2) = 0;
        end
        for i = 1:size(image,1)
            for j = 1:size(image,2)
                min_distance = abs(image(i,j)-m(1,1));
                min_distance_class = 1;
                for k=2:KMEANS_K
                    if abs(image(i,j)-m(k,1)) < min_distance
                        min_distance = abs(image(i,j)-m(k,1));
                        min_distance_class = k;
                    end
                end
                m(min_distance_class,2) = m(min_distance_class,2)+1;
                m(min_distance_class,3) = (m(min_distance_class,2)-1)/m(min_distance_class,2)*m(min_distance_class,3) + image(i,j)/m(min_distance_class,2);
            end
        end
        for i = 1:KMEANS_K
            m(i,1) = m(i,3);
        end
    end
    max_cnt = [-1,-1,-1];
    max_cnt_class = [0,0,0];
    for i = 1:KMEANS_K%循环，找到样本数排2、3名的类别
        if m(i,1)>max_cnt(1)
            max_cnt(3) = max_cnt(2);
            max_cnt_class(3) = max_cnt_class(2);
            max_cnt(2) = max_cnt(1);
            max_cnt_class(2) = max_cnt_class(1);
            max_cnt(1) = m(i,1);
            max_cnt_class(1) = i;
        elseif m(i,1)>max_cnt(2)
            max_cnt(3) = max_cnt(2);
            max_cnt_class(3) = max_cnt_class(2);
            max_cnt(2) = m(i,1);
            max_cnt_class(2) = i;
        elseif m(i,1)>max_cnt(3)
            max_cnt(3) = m(i,1);
            max_cnt_class(3) = i;
        end
    end
    
    value = uint8(0.5*(m(max_cnt_class(2),1)+m(max_cnt_class(3),1)));
end

