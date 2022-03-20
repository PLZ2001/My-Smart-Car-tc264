function [class_Name] = ClassificationV3(image)
%[class_Name] = Classification(image)
%image：二值化+逆透视后的图片
%class_Name：赛道类别字符串
    BayesTable = [1.685 -3.593 1.532 -4.908 9.487 -2.020
                13.395 16.396 12.648 16.239 13.193 9.854
                9.275 9.163 63.356 62.429 7.743 66.090
                8.719 7.773 6.405 4.235 5.764 0.084
                2.974 7.324 1.222 6.714 13.834 3.711
                6.857 10.178 10.562 14.006 19.157 12.922
                -11.146 -14.156 -23.473 -26.963 -38.984 -28.164
                68.947 68.188 103.941 103.082 95.119 113.900
                -17.427 -15.530 -30.721 -25.538 -39.677 -30.052
                8.645 6.568 13.853 9.487 18.106 12.077
                34.226 -8.819 57.349 10.286 87.398 98.488
                -134.646 -130.190 -46.101 -41.610 -33.127 -37.576
                -1.891 35.422 10.037 56.403 84.903 96.292
                28.944 17.733 56.317 47.125 43.668 44.197
                621.139 621.478 643.436 630.584 554.176 585.571
                -8.681 3.995 19.472 31.779 22.297 20.645
                -254.031 -254.603 -427.897 -419.047 -383.225 -467.807];

    arg = Get25(image);
    classification_Data = zeros(1,6);
    classification_Data_max1 = 0;
    classification_Result1 = 0;
    classification_Data_max2 = 0;
    classification_Result2 = 0;
    for i = 1:6
        classification_Data(i) = arg(1)*BayesTable(1,i)+arg(2)*BayesTable(2,i)+arg(3)*BayesTable(3,i)+arg(4)*BayesTable(4,i)+arg(5)*BayesTable(5,i)+arg(6)*BayesTable(6,i)+arg(7)*BayesTable(7,i)+arg(8)*BayesTable(8,i)+arg(9)*BayesTable(9,i)+arg(10)*BayesTable(10,i)+arg(12)*BayesTable(11,i)+arg(13)*BayesTable(12,i)+arg(14)*BayesTable(13,i)+arg(17)*BayesTable(14,i)+arg(18)*BayesTable(15,i)+arg(19)*BayesTable(16,i)+BayesTable(17,i);
                    
        if classification_Data(i)>classification_Data_max1
            classification_Data_max2 = classification_Data_max1;
            classification_Result2 = classification_Result1;
            classification_Data_max1 = classification_Data(i);
            classification_Result1 = i;           
        elseif classification_Data(i)>classification_Data_max2
            classification_Data_max2 = classification_Data(i);
            classification_Result2 = i;
        end
    end
    class_Name_Group = {'左弯','右弯','左环岛', '右环岛','三岔路口','十字路口'};
    if classification_Data_max1-classification_Data_max2 > 35
        class_Name = class_Name_Group(classification_Result1);
    else
        class_Name = '未知';
    end
end

