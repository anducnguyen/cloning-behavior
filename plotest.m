clear clr
figure;

clr.r = [0.850 0.325 0.098];
clr.p = [0.494 0.184 0.556];
clr.y = [0.929 0.694 0.125];
clr.b = [0 0.447 0.741];
clr.g = [0.466 0.674 0.188];

scatter3(kdb_raw(class==1),range_raw(class==1),range_rate_raw(class==1),10,clr.g , 'Marker','o','LineWidth',1);
 hold on
 scatter3(kdb_raw(class==2),range_raw(class==2),range_rate_raw(class==2),10,clr.r , 'Marker','o','LineWidth',1);
 scatter3(kdb_raw(class==3),range_raw(class==3),range_rate_raw(class==3),10,clr.y , 'Marker','o','LineWidth',1);
 scatter3(kdb_raw(class==4),range_raw(class==4),range_rate_raw(class==4),10,clr.b , 'Marker','o','LineWidth',1);
 scatter3(kdb_raw(class==5),range_raw(class==5),range_rate_raw(class==5),10,clr.p , 'Marker','o','LineWidth',1);
xlabel('kdb Raw');
ylabel('Range Raw');
zlabel('Range Rate Raw');
title('Driver 7 clustered')
 rotate3d on
 %%
 beta = [beta1';beta2';beta3';beta4'];
 fprintf('\nVaribles of mode 1 are: \n');
 fprintf(' %d  ',data_table1(estimated_model_index_mode1).binary_table);
 fprintf('\nVaribles of mode 2 are: \n');
 fprintf(' %d  ',data_table2(estimated_model_index_mode2).binary_table);
 fprintf('\nVaribles of mode 3 are: \n');
 fprintf(' %d  ',data_table3(estimated_model_index_mode3).binary_table);
 fprintf('\nVaribles of mode 4 are: \n');
 fprintf(' %d  ',data_table4(estimated_model_index_mode4).binary_table);
 fprintf('\nVaribles of mode 5 are: \n');
%  fprintf(' %d  ',data_table5(estimated_model_index_mode5).binary_table);
%%
 beta_cell = num2cell(beta);

beta_cell(1,10) = {'purple'};
beta_cell(2,10) = {'green'};
beta_cell(3,10) = {'red'};
% beta_cell(4,10) = {'green'};
%  beta_cell(5,10) = {'green'};
%%
% svm_beta = [Mdl_XfullD_1.Beta';Mdl_XfullD_2.Beta';Mdl_XfullD_3.Beta'];
svm_beta = [Mdl2.Beta';Mdl3.Beta';Mdl4.Beta';Mdl5.Beta';] ;
 %%
%  hold on
%  svm_3d_plot(Mdl_XfullD_1,X_fullD,theclass1)
%  hold on
%  svm_3d_plot(Mdl_XfullD_2,X_fullD,theclass2)
% hold on
%  svm_3d_plot(Mdl_XfullD_3,X_fullD,theclass3)
%  hold on
% svm_3d_plot(Mdl_XfullD_4,X_fullD,theclass4)

%  hold on
%  svm_3d_plot(Mdl_3D1,X3D,theclass1)
 hold on
 svm_3d_plot(Mdl2,X2D2,theclass2)
hold on
 svm_3d_plot(Mdl3,X2D2,theclass3)
 hold on
svm_3d_plot(Mdl4,X2D2,theclass4)


title('Driver 1 Clustered with Decision Boundary');


%% test prediction
%predict(Mdl5,[data_table1(255).model_variables_matrices(1,6) data_table1(255).model_variables_matrices(1,4) data_table1(255).model_variables_matrices(1,5)])


table = {'misclass' misclass1 misclass2 misclass3 misclass4; 'accuracy' accuracy1 accuracy2 accuracy3 accuracy4}