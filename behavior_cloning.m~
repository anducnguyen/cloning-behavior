% This file is containing the code for
% 1. clustering and obtaining variable's parameter
% 2. find decision boundaries and its parameter - SVM
% 3. create front car profile (based on one of driver data)
% and use the trained model's params & svm's params to follow the created profile


%% 1. clustering and obtaining variable's parameter

load('driver1_data1.mat')
load ('driver2_data1.mat');
load ("driver3_data1.mat");
load ("driver4_data1.mat");
load ("driver5_data1.mat");
load ("driver6_data1.mat");
load ("driver7_data1.mat");
%%
driver_data_raw(:,:) =[driver1_data1(1:length(driver1_data1)/3,:);driver2_data1(1:length(driver2_data1)/3,:);driver3_data1(1:length(driver3_data1)/3,:);driver4_data1(1:length(driver4_data1)/3,:);driver5_data1(1:length(driver5_data1)/3,:);driver6_data1(1:length(driver6_data1)/3,:);driver7_data1(1:length(driver7_data1)/3,:)];
driver_data_normalize = zeros(size(driver_data_raw));

for i = 1:1:14
driver_data_normalize(:,i) = 2*((driver_data_raw(:,i)-min(driver_data_raw(:,i)))/(max(driver_data_raw(:,i))-min(driver_data_raw(:,i))))-1;
end

acc = driver_data_normalize(1:size(driver_data_normalize(:,6))-4,6);
acc_tminus1 = driver_data_normalize(2:size(driver_data_normalize(:,6))-3,6);
acc_tminus2 = driver_data_normalize(3:size(driver_data_normalize(:,6))-2,6);
acc_tminus3 = driver_data_normalize(4:size(driver_data_normalize(:,6))-1,6);

speed  = driver_data_normalize(1:size(driver_data_normalize(:,5))-4,5);
speed_tminus1 = driver_data_normalize(2:size(driver_data_normalize(:,5))-3,5);
speed_tminus2 = driver_data_normalize(3:size(driver_data_normalize(:,5))-2,5);
speed_tminus3 = driver_data_normalize(4:size(driver_data_normalize(:,5))-1,5);

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
speed_tminus3_raw = driver_data_raw(4:size(driver_data_raw(:,5))-1,5);

frontcar_acc_raw = driver_data_raw(1:size(driver_data_raw(:,10))-4,10);
frontcar_speed_raw = driver_data_raw(1:size(driver_data_raw(:,9))-4,9);
range_raw = driver_data_raw(1:size(driver_data_raw(:,7))-4,7);
range_rate_raw = driver_data_raw(1:size(driver_data_raw(:,8))-4,8);
kdb_raw = driver_data_raw(1:size(driver_data_raw(:,11))-4,11);
jerk_raw = driver_data_raw(1:size(driver_data_raw(:,12))-4,12);
invTTC_raw = driver_data_raw(1:size(driver_data_raw(:,13))-4,13);
THW_raw = driver_data_raw(1:size(driver_data_raw(:,14))-4,14);


y  = acc;
y_raw = acc_raw;
offset_matrix = ones(length(y),1);
phi = [acc_tminus3 frontcar_speed frontcar_acc range range_rate kdb invTTC THW];

phi_raw = [acc_tminus3_raw frontcar_speed_raw frontcar_acc_raw range_raw range_rate_raw kdb_raw invTTC_raw THW_raw];

opt_f.c = 10000;
opt_f.rmv_const = true; 

% opt_f.calc_r = true;  
% opt_f.calc_ir = true;
% opt_f.calc_spr = true;
% opt_f.calc_w = true;

[gLDs, LDs] = ohpk_pwarx_data2feature_space( phi, y, opt_f );

% figure
% E = evalclusters(gLDs,'kmeans','DaviesBouldin','KList',[1:10]);
% plot(E)

mode_num = 5;
%mode_num = E.OptimalK
opt.NumOfInitialValues = 10000;   
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
beta1= beta1*data_table1(estimated_model_index_mode1).binary_table;
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

%% 2. 

%clr = lines(mode_num);
%figure;
% scatter(range kdb,10, clr(class1,:), 'Marker','o','LineWidth',1);
%hold on

phi_train1(:,4)= range(:,:);

% classes = num2cell(class);
% MdlGau =
% fitrsvm(phi,class,'Standardize',true,'KFold',5,'KernelFunction','gaussian')
%class = class1(:,:);
%% support vector 1 agaist 2|3|4|5
theclass1 = zeros(size(phi_train1(:,4),1),1);
theclass1(class==2) = -1;
theclass1(class==3) = -1;
theclass1(class==4) = -1;
theclass1(class==5) = -1;
theclass1(class==1)= 1;

X3D = [kdb range range_rate];
% X_fullD = [acc_tminus3 frontcar_speed frontcar_acc range range_rate kdb invTTC THW];
%Mdl = fitcsvm(X2D,theclass,'KernelFunction','mysigmoid','Standardize',true); 

Mdl_3D1 = fitcsvm(X3D,theclass1,'KernelFunction','linear','OptimizeHyperparameters','auto',...
      'HyperparameterOptimizationOptions',struct('AcquisitionFunctionName',...
      'expected-improvement-plus','ShowPlots',false));
  %%
CVMdl_3D1 = crossval(Mdl_3D1);
misclass3D1 = kfoldLoss(CVMdl_3D1);
misclass3D1
%%
% d = 0.02; % Step size of the grid
% [x1Grid1,x2Grid1] = meshgrid(min(X2D(:,1)):d:max(X2D(:,1)),...
%     min(X2D(:,2)):d:max(X2D(:,2)));
% xGrid1 = [x1Grid1(:),x2Grid1(:)];        % The grid
% [~,scores1] = predict(Mdl1,xGrid1); % The scores
% 
% figure;
% %h(1:2) = gscatter(X2D(:,1),X2D(:,2),theclass1);
% %hold on
% %h(3) = plot(X2D(Mdl.IsSupportVector,1),...
% %    X2D(Mdl.IsSupportVector,2),'ko','MarkerSize',10);
%     % Support vectors
% contour(x1Grid1,x2Grid1,reshape(scores1(:,2),size(x1Grid1)),[0 0],'k');
%     % Decision boundary
% %title('Scatter Diagram with the Decision Boundary')
% %legend({'-1','1','Support Vectors'},'Location','Best');
% %hold off
% hold on

%% support vector 2 agaist 1|3
theclass2 = zeros(size(phi_train1(:,4),1),1);
theclass2(class==1) = -1;
theclass2(class==3) = -1;
theclass2(class==4) = -1;
theclass2(class==5) = -1;
theclass2(class==2)= 1;

%Mdl = fitcsvm(X2D,theclass,'KernelFunction','mysigmoid','Standardize',true); 

Mdl_XfullD_2 = fitcsvm(X_fullD,theclass2,'KernelFunction','linear','OptimizeHyperparameters','auto',...
      'HyperparameterOptimizationOptions',struct('AcquisitionFunctionName',...
      'expected-improvement-plus','ShowPlots',false));
CVMdl_XfullD_2 = crossval(Mdl_XfullD_2);
misclass2 = kfoldLoss(CVMdl_XfullD_2);
misclass2
 %%
% d = 0.02; % Step size of the grid
% [x1Grid2,x2Grid2] = meshgrid(min(X2D(:,1)):d:max(X2D(:,1)),...
%     min(X2D(:,2)):d:max(X2D(:,2)));
% xGrid2 = [x1Grid2(:),x2Grid2(:)];        % The grid
% [~,scores2] = predict(Mdl2,xGrid2); % The scores
% 
% %figure;
%h(1:2) = gscatter(X2D(:,1),X2D(:,2),theclass1);
%hold on
%h(3) = plot(X2D(Mdl.IsSupportVector,1),...
%    X2D(Mdl.IsSupportVector,2),'ko','MarkerSize',10);
    % Support vectors
%contour(x1Grid2,x2Grid2,reshape(scores2(:,2),size(x1Grid2)),[0 0],'k');
    % Decision boundary
%title('Scatter Diagram with the Decision Boundary')
%legend({'-1','1','Support Vectors'},'Location','Best');
%hold off
hold on
%% support vector 3 agaist 1|2
theclass3 = zeros(size(phi_train1(:,4),1),1);
theclass3(class==1) = -1;
theclass3(class==2) = -1;
theclass3(class==4) = -1;
theclass3(class==5) = -1;
theclass3(class==3)= 1;


%Mdl = fitcsvm(X2D,theclass,'KernelFunction','mysigmoid','Standardize',true); 

Mdl_XfullD_3 = fitcsvm(X_fullD,theclass3,'KernelFunction','linear','OptimizeHyperparameters','auto',...
      'HyperparameterOptimizationOptions',struct('AcquisitionFunctionName',...
      'expected-improvement-plus','ShowPlots',false));
CVMdl_XfullD_3 = crossval(Mdl_XfullD_3);
misclass3 = kfoldLoss(CVMdl_XfullD_3);
misclass3
%%
% d = 0.02; % Step size of the grid
% [x1Grid3,x2Grid3] = meshgrid(min(X2D(:,1)):d:max(X2D(:,1)),...
%     min(X2D(:,2)):d:max(X2D(:,2)));
% xGrid3 = [x1Grid3(:),x2Grid3(:)];        % The grid
% [~,scores3] = predict(Mdl_XfullD_1,xGrid3); % The scores

%figure;
%h(1:2) = gscatter(X2D(:,1),X2D(:,2),theclass);
%hold on
%h(3) = plot(X2D(Mdl.IsSupportVector,1),...
 %   X2D(Mdl.IsSupportVector,2),'ko','MarkerSize',10);
    % Support vectors
%contour(x1Grid3,x2Grid3,reshape(scores3(:,2),size(x1Grid3)),[0 0],'k');
    % Decision boundary
%title('Scatter Diagram with the Decision Boundary')
%legend({'-1','1','Support Vectors'},'Location','Best');
%hold off

%% support vector 4 agaist 1|2
theclass4 = zeros(size(phi_train1(:,4),1),1);
theclass4(class==1) = -1;
theclass4(class==2) = -1;
theclass4(class==3) = -1;
theclass4(class==5) = -1;
theclass4(class==4)= 1;


%Mdl = fitcsvm(X2D,theclass,'KernelFunction','mysigmoid','Standardize',true); 

Mdl_XfullD_4 = fitcsvm(X_fullD,theclass4,'KernelFunction','linear','OptimizeHyperparameters','auto',...
      'HyperparameterOptimizationOptions',struct('AcquisitionFunctionName',...
      'expected-improvement-plus','ShowPlots',false));
CVMdl_XfullD_4 = crossval(Mdl_XfullD_4);
misclass4 = kfoldLoss(CVMdl_XfullD_4);
misclass4
%% support vector 5 agaist 1|2

theclass5 = zeros(size(phi_train1(:,4),1),1);
theclass5(class==1) = -1;
theclass5(class==2) = -1;
theclass5(class==3) = -1;
theclass5(class==4) = -1;
theclass5(class==5)=1;

%[acc_tminus3 frontcar_speed frontcar_acc range range_rate kdb invTTC THW 1;


%Mdl = fitcsvm(X2D,theclass,'KernelFunction','mysigmoid','Standardize',true); 

Mdl_XfullD_5 = fitcsvm(X_fullD,theclass5,'KernelFunction','linear','OptimizeHyperparameters','auto',...
      'HyperparameterOptimizationOptions',struct('AcquisitionFunctionName',...
      'expected-improvement-plus','ShowPlots',false));
CVMdl_XfullD_5 = crossval(Mdl_XfullD_5);
misclass5 = kfoldLoss(CVMdl_XfullD_5);
misclass5


%% 3. 

front_v = driver1_data1(850:1250,9);
n = length(front_v);
dt = 0.1;

front_x = zeros(1,n)';
front_x(1) = 15;
front_acc = zeros(n,1);
front_jerk  = zeros(n,1);

ego_v = zeros(n,1);
ego_x = front_x -10;
ego_acc = zeros(n,1);

ego_v(1:3) = front_v(1:3);

range = zeros(n,1);
range_rate = zeros(n,1);
kdb = zeros(n,1);
invTTC = zeros(n,1);
THW = zeros(n,1);

mode2_para = 0.5*ones(1,8);

mode1_para = [0.801419210029347;-0.00103929815618471;-0.112667921242722;0.143180965808151;-0.862678265012075;-0.0267755277884271;-0.840690939453128;0.201836061397930]';

for i = 2:1:n
    j = 3500+i;
front_x(i) = front_x(i-1) + front_v(i)*dt;
front_acc(i) = (front_v(i) - front_v(i-1))/dt;
front_jerk(i)= (front_acc(i)-front_acc(i-1))/dt;
if abs(front_acc(i)) >3
    front_acc(i)=front_acc(i-1);

%     if norm(regressor_matrix(i,:))>0
%         mode1 = true;
%     end
%     while mode1 == true


% end
    
end
regressor_matrix = [ego_acc front_v front_acc range range_rate kdb invTTC THW];

ego_acc(i) = mode1_para*regressor_matrix(i,:)';
ego_v(i)  = ego_acc(i)*dt +ego_v(i-1);
ego_x(i) = ego_x(i-1) + ego_v(i)*dt;
range(i) = front_x(i) - ego_x(i);
range_rate(i) = front_v(i) - ego_v(i);
invTTC(i) = range(i)/abs(front_v(i)-ego_v(i));
end

%%
figure
plot(ego_acc)
hold on
plot(front_acc)
hold off

figure
plot(ego_v)
hold on
plot(front_v)
hold off

figure 
plot(range)



