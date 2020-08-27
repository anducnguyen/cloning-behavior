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

load ("driver1_data1.mat");
front_v = driver1_data1(650:850,9);
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

