clc,clear,close all;

%��ͼƬ��Ȼ���ֵ+��͸��
A = imread('412.168����.pgm');% 48.742�󻷵�(31 ����)(32����) 56.343�һ���(21 ����)(28 ����)  8.868����(21 ��ȷ)(38 ��ȷ)
value = KmeansV2(A);
B = ImageProcessing(A>value, 40, 27.89191, 5.915322, 0.1, 2);
%ʶ������
if CheckStraightV2(B) == 1
    class_Name = 'ֱ��';
else
    class_Name = ClassificationV3(B);
end
%����ʶ�����ͽ��������߻���
[C,~,~] = DrawCenterLineV2(class_Name,B);

subplot(1 ,3 ,1),imshow(A,[0,256]);
title('ԭʼͼ')
subplot(1 ,3 ,2),imshow(B,[0,1]);
title(class_Name)
subplot(1 ,3 ,3),imshow(C,[0,4]);
title(class_Name)