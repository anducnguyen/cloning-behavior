s = drivingScenario;
roadCenters= [0 0;
            5000 0];
roadWidth=7;
road(s, roadCenters, roadWidth);

ego = vehicle(s, 'ClassID',1)
ego.Position = [0 0 0]

front = vehicle(s, 'ClassID',1)
front.Position = [15 0 0]


ego_waypoints = [ego_x zeros(n,1)];
ego_velocities = [ego_v];

front_waypoints = [front_x zeros(n,1)];
front_velocities = [front_v];
path(ego, ego_waypoints,ego_velocities);
path(front, front_waypoints,front_velocities);
plot(s,'Waypoints','on','Centerline','on');
chasePlot(ego,'Centerline','on');

s.SampleTime = 0.1
s.StopTime = 4;
while advance(s)
  pause(0.001)
end
close all

s.StopTime = 0.100;
poseRecord = record(s)

r = poseRecord(5)
r.ActorPoses(1)
r.ActorPoses(2)

close all
%%
hFigure = figure;
hFigure.Position = [1000 500 2000 1000];

hPanel1 = uipanel(hFigure,'Units','Normalized','Position',[0 1/4 1/2 3/4],'Title','Scenario Plot');
hPanel2 = uipanel(hFigure,'Units','Normalized','Position',[0 0 1/2 1/4],'Title','Chase Plot');
hPanel3 = uipanel(hFigure,'Units','Normalized','Position',[1/2 0 1/2 1],'Title','Bird''s-Eye Plot');

hAxes1 = axes('Parent',hPanel1);
hAxes2 = axes('Parent',hPanel2);
hAxes3 = axes('Parent',hPanel3);


plot(s, 'Parent', hAxes1, 'Centerline','on');
chasePlot(ego, 'Parent', hAxes2,'Centerline','on', 'ViewHeight', 5,'ViewLocation', [-20 15]);

%%assign a bird's-eye plot in third axes.
egoCarBEP = birdsEyePlot('Parent',hAxes3,'XLimits',[0 300],'YLimits',[-20 20]);
fastTrackPlotter = trackPlotter(egoCarBEP,'MarkerEdgeColor','red','DisplayName','front','VelocityScaling',.5);
egoTrackPlotter = trackPlotter(egoCarBEP,'MarkerEdgeColor','blue','DisplayName','ego','VelocityScaling',.5);
egoLanePlotter = laneBoundaryPlotter(egoCarBEP);
plotTrack(egoTrackPlotter, [0 0]);
egoOutlinePlotter = outlinePlotter(egoCarBEP);

restart(s)
s.StopTime = Inf;

while advance(s)
    t = targetPoses(ego);
    plotTrack(fastTrackPlotter, t.Position, t.Velocity);
    rbs = roadBoundaries(ego);
    plotLaneBoundary(egoLanePlotter, rbs);
    [position, yaw, length, width, originOffset, color] = targetOutlines(ego);
    plotOutline(egoOutlinePlotter, position, yaw, length, width, 'OriginOffset', originOffset, 'Color', color);
end