function [class_Name] = Classification(image)
%[class_Name] = Classification(image)
%image：二值化+逆透视后的图片
%class_Name：赛道类别字符串
    BayesTable = [-6.212 -9.319 -24.312 -25.426 -3.056 -29.253
                25.361 26.676 57.032 54.606 25.934 60.921
                30.833 28.652 55.679 57.031 30.260 59.570
                -11.965 -9.801 -27.646 -27.080 -5.181 -31.199
                -7.657 -8.620 -9.958 -20.068 1.053 -2.612
                20.082 7.303 33.639 33.028 19.800 30.603
                -7.004 11.375 13.713 21.002 4.998 17.201
                -3.465 -3.579 -12.959 -6.872 5.111 0.053
                155.160 54.548 219.792 151.069 241.736 245.156
                52.537 147.741 143.563 208.364 230.583 233.707
                138.356 156.841 144.860 165.326 99.307 119.594
                122.261 105.980 128.375 112.617 70.663 89.646
                -189.763 -187.540 -328.932 -331.373 -322.336 -380.283];

    arg = Get16(image);
    classification_Data = zeros(1,6);
    for i = 1:6
        classification_Data(i) = arg(1)*BayesTable(1,i)+arg(2)*BayesTable(2,i)+arg(3)*BayesTable(3,i)+arg(4)*BayesTable(4,i)+arg(5)*BayesTable(5,i)+arg(6)*BayesTable(6,i)+arg(7)*BayesTable(7,i)+arg(8)*BayesTable(8,i)+arg(10)*BayesTable(9,i)+arg(11)*BayesTable(10,i)+arg(14)*BayesTable(11,i)+arg(15)*BayesTable(12,i)+BayesTable(13,i);
        if i==1
            classification_Data_max = classification_Data(1);
            classification_Result = 1;
        elseif classification_Data(i)>classification_Data_max
            classification_Data_max = classification_Data(i);
            classification_Result = i;         
        end
    end
    class_Name_Group = {'左弯','右弯','左环岛', '右环岛','三岔路口','十字路口'};
    class_Name = class_Name_Group(classification_Result);
end

