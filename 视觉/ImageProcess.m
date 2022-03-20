clc,clear,close all;

%读图片，然后二值+逆透视
A = imread('412.168靠右.pgm');% 48.742左环岛(31 错误)(32错误) 56.343右环岛(21 错误)(28 错误)  8.868靠右(21 正确)(38 正确)
value = KmeansV2(A);
B = ImageProcessing(A>value, 40, 27.89191, 5.915322, 0.1, 2);
%识别类型
if CheckStraightV2(B) == 1
    class_Name = '直道';
else
    class_Name = ClassificationV3(B);
end
%根据识别类型进行中心线绘制
[C,~,~] = DrawCenterLineV2(class_Name,B);

subplot(1 ,3 ,1),imshow(A,[0,256]);
title('原始图')
subplot(1 ,3 ,2),imshow(B,[0,1]);
title(class_Name)
subplot(1 ,3 ,3),imshow(C,[0,4]);
title(class_Name)