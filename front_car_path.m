%% create front-car path
clear

% '1   time';
% '2   throttle';
% '3   brake'; 
% '4   steer'; 
% '5   speed'; 
% '6   acceleration'; 
% '7   range'; 
% '8   range_rate';
% '9   lead_car_velocity';
% '10  lead_car_acc';
% '11  kdb';
% '12  jerk';
% '13  TTC_inverse';
% '14  THW' ];

load('driver2_data1.mat');
load('driver2_clustered_svm_mode1234_fullD.mat','Mdl_XfullD_1');
load('driver2_clustered_svm_mode1234_fullD.mat','Mdl_XfullD_2');
load('driver2_clustered_svm_mode1234_fullD.mat','Mdl_XfullD_3');
load('driver2_clustered_svm_mode1234_fullD.mat','Mdl_XfullD_4');
load('driver2_clustered_svm_mode1234_fullD.mat','Mdl_XfullD_5');


front_v = driver2_data1(650:1850,9);
n = length(front_v);
dt = 0.1;

front_x = zeros(1,n)';
front_x(1) = 15;
front_acc = zeros(n,1);

ego_v = zeros(n,1);
ego_x = zeros(1,n);
ego_x(1) = front_x(1) -10;
ego_acc = zeros(n,1);

ego_v(1:3) = front_v(1:3);
for i = 1:1:2
ego_acc(i) = ego_v(i+1) - ego_v(i);
end

range = zeros(n,1);
range_rate = zeros(n,1);
kdb = zeros(n,1);
invTTC = zeros(n,1);
THW = zeros(n,1);


mode1_para = [0.777674977189423;-0.060562978033250;0.015267715116141;0.012000473144785;0.147732906154473;-0.010555766071335;0.117129420849696;-0.073881765568356]';
mode2_para = [0.587899784589997;-0.034437539193745;0.010290568527205;0.052496188225237;-0.052711605808447;-0.116978286465032;-0.286892174837057;-0.119968902276729]';
mode3_para = [0.928536321383595;0.050680957727106;-2.049936663322427e-04;-0.002187110516528;-0.531135835555672;-0.013082231772931;-0.590493205727875;0.113673424021394]';
mode4_para = [0.427319837053940;-0.041648189978660;0.010848162307127;-0.004343181531193;-0.217785688227683;-0.020382513368423;-0.130669590135085;-0.030693354313629]';
mode5_para = [0.882704332588087;0.015076437718191;-0.006792991534577;8.477097575989023e-04;-0.044483323662678;0.001630557209259;-0.104847301799530;-0.019363687871173]';


for i = 2:1:n

front_x(i) = front_x(i-1) + front_v(i)*dt;
front_acc(i) = (front_v(i) - front_v(i-1))/dt;


phi = [ego_acc front_v front_acc range range_rate kdb invTTC THW];

if predict(Mdl_XfullD_1, phi(i,:)) == 1
ego_acc(i) = mode3_para*phi(i,:)';
elseif predict(Mdl_XfullD_2, phi(i,:)) == 1
ego_acc(i) = mode2_para*phi(i,:)';
elseif predict(Mdl_XfullD_3, phi(i,:)) == 1
ego_acc(i) = mode4_para*phi(i,:)';
elseif predict(Mdl_XfullD_4, phi(i,:)) == 1
ego_acc(i) = mode5_para*phi(i,:)';
elseif predict(Mdl_XfullD_5, phi(i,:)) == 1
ego_acc(i) = mode1_para*phi(i,:)';
end
    
ego_v(i)  = ego_acc(i)*dt +ego_v(i-1);
ego_x(i) = ego_x(i-1) + ego_v(i)*dt;

range(i) = front_x(i) - ego_x(i);
range_rate(i) = front_v(i) - ego_v(i);
if range_rate(i)>0
    kdb(i) = -10*log(abs(-4*10e-9*range(i)/ego_v(i)));
elseif range_rate(i)<0
    kdb(i) = 10*log(abs(-4*10e-9*range(i)/ego_v(i)));
end
jerk(i) = ego_acc(i)/dt;
invTTC(i) =ego_v(i)/range(i);
THW(i) = (front_v(i) - ego_v(i))/range(i);
end

%%
figure
plot(ego_acc)
hold on
plot(front_acc)
legend('ego acc','front acc')
hold off

figure
plot(ego_v)
hold on
plot(front_v)
legend('ego v','front v')
hold off

figure 
plot(range)

%%
t= 0;
while t<60
for i=2:1:n
if ego_v(i) > 6.7
    acc(i) = mode2_para*regressor_matrix(i,:)';
elseif ego_v<6.7
    acc(i) = mode1_para*regressor_matrix(i,:)';


end
t = t+0.1;
figure(1)
plot(acc(i),'-o','MarkerSize',5,'MarkerFaceColor','r')
pause(0.1)
end
end
%%
x = [12 64 24];
%disp('what graph you want to draw?')
plottype = input('what graph you want to draw?','s');

switch plottype
    case 'bar' 
        bar(x)
        title('Bar Graph')
    case {'pie','pie3'}
        pie3(x)
        title('Pie Chart')
    otherwise
        warning('Unexpected plot type. No plot created.')
end

%%

b=1.5;
bact=zeros(1,12);
n=1;
while n<13
  b=b*2;
  bact(n)=b;
  n=(n)+1;
  fprintf('\n%2d',b);
end

