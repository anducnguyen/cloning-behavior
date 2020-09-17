clear
%%
load('driver1_data1.mat')
load ('driver2_data1.mat');
load ("driver3_data1.mat");
load ("driver4_data1.mat");
load ("driver5_data1.mat");
load ("driver6_data1.mat");
load ("driver7_data1.mat");
%%
driver_data_raw(:,:) =[driver1_data1(:,:);driver2_data1(:,:);driver3_data1(:,:)];
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


%% assign y - phiload ("driver2_data1.mat");


y  = acc;
y_raw = acc_raw;
offset_matrix = ones(length(y),1);
phi = [acc_tminus3 frontcar_speed frontcar_acc range range_rate kdb invTTC THW];

phi_raw = [acc_tminus3_raw frontcar_speed_raw frontcar_acc_raw range_raw range_rate_raw kdb_raw invTTC_raw THW_raw];
%% estimate feature vectors

opt_f.c = 10000;
opt_f.rmv_const = true; 

% following four options are default settings of calculation.
% opt_f.calc_r = true;  
% opt_f.calc_ir = true;
% opt_f.calc_spr = true;
% opt_f.calc_w = true;

% calculate the feature vectors through the dynamics.
[gLDs, LDs] = ohpk_pwarx_data2feature_space( phi, y, opt_f );
% %
% figure
% E = evalclusters(gLDs,'kmeans','DaviesBouldin','KList',[1:10]);
% plot(E)

%% clustering

mode_num = 5;
%mode_num = E.OptimalK
opt.NumOfInitialValues = 20000;   
% opt.MaxRepetations = 500;       
opt.CenterInitializeMethod = 'pickout';    
% opt.CenterInitializeMethod = 'normal';    
% opt.CenterInitializeMethod = 'uniform';    
% opt.CenterInitializeStd = std(gLDs);    
% opt.CenterInitializeMean = mean(gLDs,1);    

tic;
opt.ShowProgress = 't';
opt.ShowProgressSkip = 100;
[center, class] = ohpk_pwarx_weighted_kmeans(gLDs, mode_num, LDs, opt);
toc;

%%
%perate mode
for i = 1:mode_num
    data(i).ymode = y(class==i); 
    data(i).phimode = phi(class==i, :); 
end

%%
 %mode 1

% create binary table
number_of_variable = size(data(1).phimode,2);
binary_table = dec2bin(1:(2^(number_of_variable)-1)) - '0';

for i = 1:1:2^(number_of_variable)-1;
data_table1(i).mode_index = i;
data_table1(i).binary_table = [binary_table(i,:)];
end
 [row col] = find(binary_table);
  %calculate aic
for i= 1:1:size(binary_table,1);
   selected_array1 = data(1).phimode(:,col(row == i));
   data_table1(i).model_variables_matrices = selected_array1(:,:);
   [data_table1(i).theta,~,~,~,data_table1(i).logL] = mvregress(data_table1(i).model_variables_matrices ,data(1).ymode);
   aicmode1(i) = aicbic(data_table1(i).logL, size(data_table1(i).model_variables_matrices,2));
   invaicmode1(i) = 1/aicmode1(i);
end
[aic_value_mode1 estimated_model_index_mode1] = min(aicmode1);

beta1 = inv(data(1).phimode'*diag(LDs.w(class==1))*data(1).phimode)*data(1).phimode'*diag(LDs.w(class==1))*data(1).ymode;

fprintf('\nVaribles of mode 1 are: \n');
fprintf(' %d  ',data_table1(estimated_model_index_mode1).binary_table);
fprintf('\n');
fprintf('With parameters: \n');
%fprintf(' %f  ',data_table1(estimated_model_index_mode1).theta);
fprintf(' %f  ',beta1);
fprintf('\n');
%%
%mode 2

% create binary table
number_of_variable = size(data(2).phimode,2);
binary_table = dec2bin(1:(2^(number_of_variable)-1)) - '0';

for i = 1:1:2^(number_of_variable)-1;
data_table2(i).mode_index = i;
data_table2(i).binary_table = [binary_table(i,:)];
end
 [row col] = find(binary_table);
   

%calculate aic
for i= 1:1:size(binary_table,1);
   selected_array2 = data(2).phimode(:,col(row == i));
   data_table2(i).model_variables_matrices = selected_array2(:,:);
   [data_table2(i).theta,~,~,~,data_table2(i).logL] = mvregress(data_table2(i).model_variables_matrices ,data(2).ymode);
   aicmode2(i) = aicbic(data_table2(i).logL, size(data_table2(i).model_variables_matrices,2));
end
[aic_value_mode2 estimated_model_index_mode2] = min(aicmode2);

beta3 = pinv(data(2).phimode'*diag(LDs.w(class==2))*data(2).phimode)*data(2).phimode'*diag(LDs.w(class==2))*data(2).ymode;

fprintf('\nVaribles of mode 3 are: \n');
fprintf(' %d  ',data_table2(estimated_model_index_mode2).binary_table);
fprintf('\n');
fprintf('With parameters: \n');
%fprintf(' %f  ',data_table2(estimated_model_index_mode2).theta);
fprintf(' %f  ',beta3);
fprintf('\n');

%%
 %mode 3

% create binary table
number_of_variable = size(data(3).phimode,2);
binary_table = dec2bin(1:(2^(number_of_variable)-1)) - '0';

for i = 1:1:2^(number_of_variable)-1;
data_table3(i).mode_index = i;
data_table3(i).binary_table = [binary_table(i,:)];
end
 [row col] = find(binary_table);
   

%calculate aic
for i= 1:1:size(binary_table,1);
   selected_array3 = data(3).phimode(:,col(row == i));
   data_table3(i).model_variables_matrices = selected_array3(:,:);
   [data_table3(i).theta,~,~,~,data_table3(i).logL] = mvregress(data_table3(i).model_variables_matrices ,data(3).ymode);
   aicmode3(i) = aicbic(data_table3(i).logL, size(data_table3(i).model_variables_matrices,2));
end
[aic_value_mode3 estimated_model_index_mode3] = min(aicmode3);

beta2 = pinv(data(3).phimode'*diag(LDs.w(class==3))*data(3).phimode)*data(3).phimode'*diag(LDs.w(class==3))*data(3).ymode;
fprintf('\n Varibles of mode 2 are: \n');

fprintf(' %d  ',data_table3(estimated_model_index_mode3).binary_table);
fprintf('\n');
fprintf('With parameters: \n');
%fprintf(' %f ',data_table3(estimated_model_index_mode3).theta);
fprintf(' %f  ',beta2);
fprintf('\n');



%%
% plot data after cluster
clr = lines(mode_num);
figure('name', 'Clustered Data'), hold on
% scatter3(phi(:,1)', phi(:,2)',y(:,:)', 10, 'Marker','d','LineWidth',1);
scatter3(data(2).phimode(:,1), data(2).phimode(:,4),data(2).ymode(:,:), 'Marker','d', 'LineWidth',1)
hold off
view(3), axis vis3d, box on, rotate3d on
xlabel('x(1)'), ylabel('x(2)'), zlabel('y');
grid on
set(gca,'Fontsize',11)
set(gcf,'Position',[10 10 600 400])
%%
% plot data after cluster
clr = lines(mode_num);
figure('name', 'Clustered Data'), hold on
% scatter3(phi(:,1), phi(:,2),y(:,1), 10, clr(class,:), 'Marker','d','LineWidth',1);
scatter3(gLDs(:,19), gLDs(:,8),gLDs(:,3),10, clr(class,:), 'LineWidth',1)
hold off
view(3), axis vis3d, box on, rotate3d on
xlabel('medium vector'), ylabel('parameter vector 1'), zlabel('parameter vector ');

view([0 90]);grid on
set(gca,'Fontsize',14)
set(gcf,'Position',[10 10 600 400])

%%
% plot data after cluster
clr = lines(mode_num);
figure('name', 'Clustered Data'), hold on
scatter3(gLDs(:,20), gLDs(:,9),gLDs(:,6),50, clr(class,:), 'Marker','.','LineWidth',1)
% scatter3(center(:,1), center(:,2), center(:,3), 100, clr, 'Marker','o', 'LineWidth',1)
hold off
view([0 0]), axis vis3d, box on, rotate3d on
%xlabel('range'), ylabel('kdb'), zlabel('jerk');
grid on
% set(gca,'Fontsize',13)
% set(gcf,'Position',[50 50 600 400])
% clear gLDs n_thottle



%%
cost_func = "NRMSE";
yref = n_speed(class==1);
beta = pinv(data(2).phimode'*data(2).phimode)*data(2).phimode'*data(2).ymode
Y = data(2).phimode*[0 0 -1.2 -7 0 3]';
Ynew = data(2).phimode*beta3;
fit1 = goodnessOfFit(y3, y3ref,cost_func)
fit2 = goodnessOfFit(y3new, y3ref,cost_func)



plot([1:1:size(yref)],y3ref,'Marker','o', 'LineWidth',2,'Color','r')
hold on
plot([1:1:size(yref)],y3,'Marker','diamond', 'LineWidth',1,'Color','y')
plot([1:1:size(yref)],y3new,'Marker','x', 'LineWidth',0.1, 'Color','k')
