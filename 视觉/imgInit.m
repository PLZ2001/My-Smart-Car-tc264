clear

filename=dir('D:\OneDrive\����\class4\class4\����·��\*.pgm');%�洢�����ļ���
for i=1:length(filename)
    fname=strcat('D:\OneDrive\����\class4\class4\����·��\',filename(i).name);%�洢·��
    file=[filename(i).name];
    file(end-3:end)=[];%ȥ���ַ�������λ
    
    A=imread(fname);%����
    
    value = Kmeans(A);
    B = ImageProcessing(A(:,1:end-1)>value, 40, 27.89191, 5.915322, 0.1, 2);
       
    
    if CheckStraightV2(B) == 1
        class_Name = 'ֱ��';
    else
        class_Name = Classification(B);
    end
    %����ʶ�����ͽ��������߻���
    C = DrawCenterLine(class_Name,B);
    
    subplot(3 ,2 ,1),imshow(A,[0,256]);
    title('ԭʼͼ')
    subplot(3 ,2 ,2),imshow(ImageProcessing(A, 40, 27.89191, 5.915322, 0.1, 2),[0,256]);
    title('��͸�ӱ任ͼ')
    subplot(3 ,2 ,3),imshow(A>value,[0,1]);
    title('ԭʼͼ��ֵ��')
    subplot(3 ,2 ,4),imshow(B,[0,1]);
    title(class_Name)
    subplot(3 ,2 ,5),imshow(C,[0,4]);
    title(class_Name)
    
   
   
    saveas(gcf,strcat(file,'.jpg'));
end
