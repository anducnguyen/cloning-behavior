clear
%%

load('driver2_clustered_svm_mode1234fullD.mat');
load ('driver2_data1');
%%
clear driver_data_raw
driver_data_raw(:,:) =driver3_data1(1000:2000,:) ;
driver_data_normalize = zeros(size(driver_data_raw));

for i = 1:1:14

driver_data_normalize(:,i) = 2*((driver_data_raw(:,i)-min(driver_data_raw(:,i)))/(max(driver_data_raw(:,i))-min(driver_data_raw(:,i))))-1;
end

%%
acc = driver_data_normalize(1:size(driver_data_normalize(:,6))-4,6);
acc_tminus1 = driver_data_normalize(2:size(driver_data_normalize(:,6))-3,6);
acc_tminus2 = driver_data_normalize(3:size(driver_data_normalize(:,6))-2,6);
acc_tminus3 = driver_data_normalize(4:size(driver_data_normalize(:,6))-1,6);

speed  = driver_data_normalize(1:size(driver_data_normalize(:,5))-4,5);
speed_tminus1 = driver_data_normalize(2:size(driver_data_normalize(:,5))-3,5);
speed_tminus2 = driver_data_normalize(3:size(driver_data_normalize(:,5))-2,5);
%speed_tminus3 = driver_data_normalize(4:size(driver_data_normalize(:,5))-1,5);

frontcar_acc = driver_data_normalize(1:size(driver_data_normalize(:,10))-4,10);
frontcar_speed = driver_data_normalize(1:size(driver_data_normalize(:,9))-4,9);
range = driver_data_normalize(1:size(driver_data_normalize(:,7))-4,7);
range_rate = driver_data_normalize(1:size(driver_data_normalize(:,8))-4,8);
kdb = driver_data_normalize(1:size(driver_data_normalize(:,11))-4,11);
jerk = driver_data_normalize(1:size(driver_data_normalize(:,12))-4,12);
invTTC = driver_data_normalize(1:size(driver_data_normalize(:,13))-4,13);
THW = driver_data_normalize(1:size(driver_data_normalize(:,14))-4,14);


acc_raw = driver_data_raw(1:size(driver_data_raw(:,6))-4,6);
acc_tminus1_raw = driver_data_raw(2:size(driver_data_raw(:,6))-3,6);
acc_tminus2_raw = driver_data_raw(3:size(driver_data_raw(:,6))-2,6);
acc_tminus3_raw = driver_data_raw(4:size(driver_data_raw(:,6))-1,6);

speed_raw  = driver_data_raw(1:size(driver_data_raw(:,5))-4,5);
speed_tminus1_raw = driver_data_raw(2:size(driver_data_raw(:,5))-3,5);
speed_tminus2_raw = driver_data_raw(3:size(driver_data_raw(:,5))-2,5);
%speed_tminus3_raw = driver_data_raw(4:size(driver_data_raw(:,5))-1,5);

frontcar_acc_raw = driver_data_raw(1:size(driver_data_raw(:,10))-4,10);
frontcar_speed_raw = driver_data_raw(1:size(driver_data_raw(:,9))-4,9);
range_raw = driver_data_raw(1:size(driver_data_raw(:,7))-4,7);
range_rate_raw = driver_data_raw(1:size(driver_data_raw(:,8))-4,8);
kdb_raw = driver_data_raw(1:size(driver_data_raw(:,11))-4,11);
jerk_raw = driver_data_raw(1:size(driver_data_raw(:,12))-4,12);
invTTC_raw = driver_data_raw(1:size(driver_data_raw(:,13))-4,13);
THW_raw = driver_data_raw(1:size(driver_data_raw(:,14))-4,14);


%% assign y - phi
y  = acc;
y_raw = acc_raw;
% offset_matrix = ones(length(y),1);
phi = [acc_tminus3 frontcar_speed frontcar_acc range range_rate kdb invTTC THW];

phi_raw = [acc_tminus3_raw frontcar_speed_raw frontcar_acc_raw range_raw range_rate_raw kdb_raw invTTC_raw THW_raw];
%% validate mode 1

mode1  = zeros([size(phi_raw(:,:))+1]);
mode1_raw  = zeros([size(phi_raw(:,:))+1]);
for i= 1:1:size(phi(:,:),1)
predict1(i) = predict(Mdl_XfullD_1,phi(i,:));
end
label_mode1 = predict1';
for i = 1:1:size(label_mode1(:,:),1)
    if label_mode1(i) ==1
       mode1_raw(i,:) = [y_raw(i,:) phi_raw(i,:)]; 
       mode1(i,:) = [y(i,:) phi(i,:)]; 
 end
end
mode1(mode1(:,2)==0,:) = [] ;
mode1_raw(mode1_raw(:,2)==0,:) = [] ;
y_mode1 = mode1(:,1);
phi_mode1 = [mode1(:,2) mode1(:,3) mode1(:,4) mode1(:,5) mode1(:,6) mode1(:,7) mode1(:,8) mode1(:,9)];
y_est1 = phi_mode1*beta1;
accuracy1 = sqrt(sum(((phi_mode1*beta1- y_mode1).^2)'))/sqrt( sum(((y_mode1 - mean(y_mode1)).^2)'))

%% mode 2
mode2  = zeros([size(phi_raw(:,:))+1]);
mode2_raw  = zeros([size(phi_raw(:,:))+1]);
for i= 1:1:size(phi(:,:),1)
predict2(i) = predict(Mdl_XfullD_2,phi(i,:));
end
label_mode2 = predict2';
for i = 1:1:size(label_mode2(:,:),1)
    if label_mode2(i) ==1
       mode2_raw(i,:) = [y_raw(i,:) phi_raw(i,:)]; 
       mode2(i,:) = [y(i,:) phi(i,:)]; 
 end
end
mode2(mode2(:,2)==0,:) = [] ;
mode2_raw(mode2(:,2)==0,:) = [] ;
y_mode2 = mode2(:,1);
phi_mode2 = [mode2(:,2) mode2(:,3) mode2(:,4) mode2(:,5) mode2(:,6) mode2(:,7) mode2(:,8) mode2(:,9)];
y_est2 = phi_mode2*beta2;
accuracy2 = sqrt(sum(((phi_mode2*beta2- y_mode2).^2)'))/sqrt( sum(((y_mode2 - mean(y_mode2)).^2)'))

%%
mode3  = zeros([size(phi_raw(:,:))+1]);
mode3_raw  = zeros([size(phi_raw(:,:))+1]);
for i= 1:1:size(phi(:,:),1)
predict3(i) = predict(Mdl_XfullD_3,phi(i,:));
end
label_mode3 = predict3';
for i = 1:1:size(label_mode3(:,:),1)
    if label_mode3(i) ==1
       mode3_raw(i,:) = [y_raw(i,:) phi_raw(i,:)]; 
       mode3(i,:) = [y(i,:) phi(i,:)]; 
 end
end
mode3(mode3(:,2)==0,:) = [] ;
mode3_raw(mode3(:,2)==0,:) = [] ;
y_mode3 = mode3(:,1);
phi_mode3 = [mode3(:,2) mode3(:,3) mode3(:,4) mode3(:,5) mode3(:,6) mode3(:,7) mode3(:,8) mode3(:,9)];
y_est3 = phi_mode3*beta3;
accuracy3 = sqrt(sum(((phi_mode3*beta1- y_mode3).^2)'))/sqrt( sum(((y_mode3 - mean(y_mode3)).^2)'))

%%
mode4  = zeros([size(phi_raw(:,:))+1]);
mode4_raw  = zeros([size(phi_raw(:,:))+1]);
for i= 1:1:size(phi(:,:),1)
predict4(i) = predict(Mdl_XfullD_4,phi(i,:));
end
label_mode4 = predict4';
for i = 1:1:size(label_mode4(:,:),1)
    if label_mode4(i) ==1
       mode4_raw(i,:) = [y_raw(i,:) phi_raw(i,:)]; 
       mode4(i,:) = [y(i,:) phi(i,:)]; 
 end
end
mode4(mode4(:,2)==0,:) = [] ;
mode4_raw(mode4(:,2)==0,:) = [] ;
y_mode4 = mode4(:,1);

phi_mode4 = [mode4(:,2) mode4(:,3) mode4(:,4) mode4(:,5) mode4(:,6) mode4(:,7) mode4(:,8) mode4(:,9)];
y_est4 = phi_mode4*beta4;
accuracy4 = sqrt(sum(((phi_mode4*beta1- y_mode4).^2)'))/sqrt( sum(((y_mode4 - mean(y_mode4)).^2)'))

%%
% mode5  = zeros([size(phi_raw(:,:))+1]);
% mode5_raw  = zeros([size(phi_raw(:,:))+1]);
% for i= 1:1:size(phi(:,:),1)
% predict1(i) = predict(Mdl_XfullD_5,phi(i,:));
% end
% label_mode5 = predict5';
% for i = 1:1:size(label_mode5(:,:),1)
%     if label_mode5(i) ==1
%        mode5_raw(i,:) = [y_raw(i,:) phi_raw(i,:)]; 
%        mode5(i,:) = [y(i,:) phi(i,:)]; 
%  end
% end
% mode5(mode5(:,2)==0,:) = [] ;
% mode5_raw(mode5(:,2)==0,:) = [] ;
% y_mode5 = mode5(:,1);
% phi_mode5 = [mode5(:,2) mode5(:,3) mode5(:,4) mode5(:,5) mode5(:,6) mode5(:,7) mode5(:,8) mode5(:,9)];
% y_est5 = phi_mode5*beta5;
% accuracy1 = sqrt(sum(((phi_mode5*beta5- y_mode5).^2)'))/sqrt( sum(((y_mode5 - mean(y_mode5)).^2)'))
        
%%
% y_est_allmode = [y_est1 y_est2 y_est3 y_est4 y_est5]
y_est_allmode = [y_est1;y_est2;y_est3;y_est4];
y_allmode = [y_mode1;y_mode2;y_mode3;y_mode4];
figure;

plot([1:1:size(y_allmode,1)], y_allmode,'o')
hold on
plot([1:1:size(y_allmode,1)], y_est_allmode,'.')
xlabel('# of validate data');
ylabel('Output');

        title('Output agaist Prediction Driver 3 ')
        
        
        
        