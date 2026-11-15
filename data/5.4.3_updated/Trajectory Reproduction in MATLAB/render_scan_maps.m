function render_scan_maps(folder,mode,scenes,outputSuffix)
% Accumulate idealized LiDAR returns from known geometry along a flight path.
% No recorded LiDAR frames, attitude data, or sensor noise are used.
if strcmp(mode,'demo')
    if nargin < 3
        s = controlledCorridor();
    else
        s = replayCase(scenes);
    end
    scan = simulateScan(s);
    [f,h] = scanFigure(s,scan,true);
    addPlayback(f,h,scan,s);
    return
end

if nargin < 4
    outputSuffix = '';
end
cases = arrayfun(@replayCase,scenes);
for s = cases
    scan = simulateScan(s);
    f = scanFigure(s,scan,false);
    cleanup = onCleanup(@() close(f));
    output = fullfile(folder,[s.output outputSuffix '.png']);
    paperSize = f.Position(3:4);
    set(f,'InvertHardcopy','off','PaperUnits','inches','PaperSize',paperSize, ...
        'PaperPosition',[0 0 paperSize],'PaperPositionMode','manual');
    print(f,output,'-dpng','-r360','-opengl');
    fprintf('Saved %s (%d mapped points)\n',output,size(scan.points,1));
    clear cleanup
end
end

function s = controlledCorridor()
x = linspace(-8,8,401).';
y = 0.13*sin(0.82*x);
z = 0.78+0.09*cos(0.55*x);
s = struct('title','Controlled corridor | 1.0 m throat', ...
    'subtitle','Simulated LiDAR returns along an illustrative flight path', ...
    'wallNodes',[-7.5 1.5; -3 1.5; 0 0.5; 3 1.5; 7.5 1.5], ...
    'wallHeight',1.9,'wallThickness',0.5,'obstacles',zeros(0,8), ...
    'path',[x y z],'reference',zeros(0,3),'speedOnly',zeros(0,3), ...
    'start',[x(1) y(1) z(1)],'goal',[x(end) y(end) z(end)], ...
    'xLimits',[-8.6 8.6],'yLimits',[-4.1 4.1], ...
    'output','controlled_corridor_lidar_map');
end

function s = replayCase(scene)
s = struct('title',sprintf('Scene %02d | %.3f m throat',scene.scene_id,scene.width), ...
    'subtitle','Simulated LiDAR returns along the recorded Proposed trajectory', ...
    'wallNodes',scene.wall_nodes,'wallHeight',scene.wall_height, ...
    'wallThickness',scene.wall_thickness,'obstacles',scene.obstacles, ...
    'path',scene.proposed,'reference',scene.base,'speedOnly',zeros(0,3), ...
    'start',scene.start,'goal',scene.goal, ...
    'xLimits',[-16.3 16.3],'yLimits',[-5.8 5.8], ...
    'output',sprintf('scene_%02d_lidar_map',scene.scene_id));
if isfield(scene,'speed_only')
    s.speedOnly = scene.speed_only;
end
if isfield(scene,'detail_x_limits')
    s.detail = scene;
end
end

function scan = simulateScan(s)
% Resample poses by distance because the archived path has positions only.
distance = [0; cumsum(vecnorm(diff(s.path),2,2))];
[distance,idx] = unique(distance,'stable');
stations = linspace(0,distance(end),ceil(distance(end)/0.14)+1).';
origins = interp1(distance,s.path(idx,:),stations,'linear');
elevation = deg2rad(linspace(-30,25,18));
maxRange = 5;
walls = wallSegments(s);
batches = cell(size(stations));
for i = 1:numel(stations)
    % Vary the azimuth phase between consecutive sweeps.
    phase = mod((i-1)*0.61803398875,1);
    azimuth = 2*pi*((0:239).'+phase)/240;
    batches{i} = traceSweep(origins(i,:),azimuth,elevation,walls,s,maxRange);
end
points = vertcat(batches{:});
frames = repelem((1:numel(stations)).',cellfun(@(x) size(x,1),batches));
inside = points(:,1)>=s.xLimits(1) & points(:,1)<=s.xLimits(2) ...
    & points(:,2)>=s.yLimits(1) & points(:,2)<=s.yLimits(2);
points = points(inside,:);
frames = frames(inside);
% Keep the first observation in each voxel and its acquisition index.
[~,idx] = unique(round(points/0.030),'rows','stable');
scan = struct('points',points(idx,:),'frames',frames(idx), ...
    'origins',origins,'batches',{batches},'maxRange',maxRange);
end

function walls = wallSegments(s)
% Join the inner and outer edges at each end of the wall.
x = s.wallNodes(:,1);
y = s.wallNodes(:,2);
walls = zeros(0,4);
for side = [-1 1]
    outline = [[x;flipud(x);x(1)], side*[y;flipud(y)+s.wallThickness;y(1)]];
    walls = [walls; outline(1:end-1,:) outline(2:end,:)]; %#ok<AGROW>
end
end

function points = traceSweep(origin,az,el,walls,s,maxRange)
cx = cos(az);
cy = sin(az);
dx = cx*cos(el);
dy = cy*cos(el);
dz = repmat(sin(el),numel(az),1);
directions = [dx(:) dy(:) dz(:)];

% Intersect vertical walls in the horizontal plane, then check height.
% The sensor lies between the floor and wall top, so a farther wall along
% the same ray cannot restore a hit if the nearest one is outside height.
edge = walls(:,3:4)-walls(:,1:2);
offset = walls(:,1:2)-origin(1:2);
crossDE = cx*edge(:,2).'-cy*edge(:,1).';
crossDE(abs(crossDE)<1e-12) = nan;
rangeXY = (offset(:,1).*edge(:,2)-offset(:,2).*edge(:,1)).'./crossDE;
along = (cy*offset(:,1).'-cx*offset(:,2).')./crossDE;
rangeXY(~isfinite(rangeXY) | rangeXY<1e-6 | along<0 | along>1) = inf;
wallRange = min(rangeXY,[],2)./cos(el);
wallZ = origin(3)+wallRange.*dz;
wallRange(wallZ<0 | wallZ>s.wallHeight) = inf;

best = wallRange(:);
floorRange = -origin(3)./directions(:,3);
floorRange(floorRange<1e-6) = inf;
best = min(best,floorRange);

% Intersect the horizontal top faces using the stored wall footprint.
topRange = (s.wallHeight-origin(3))./directions(:,3);
xTop = origin(1)+directions(:,1).*topRange;
yTop = abs(origin(2)+directions(:,2).*topRange);
inner = interp1(s.wallNodes(:,1),s.wallNodes(:,2),xTop,'linear',nan);
validTop = topRange>1e-6 & yTop>=inner & yTop<=inner+s.wallThickness;
topRange(~validTop) = inf;
best = min(best,topRange);

o = s.obstacles;
nearby = vecnorm(o(:,2:3)-origin(1:2),2,2)<=maxRange+max(o(:,5:6),[],2);
for k = find(nearby).'
    best = min(best,obstacleRange(origin,directions,o(k,:)));
end
valid = isfinite(best) & best<=maxRange;
points = origin+directions(valid,:).*best(valid);
end

function t = obstacleRange(origin,ray,o)
% Cylinder type 1 and rotated ellipsoid type 2 use the archived geometry.
q = origin-[o(2) o(3) o(4)];
if o(1) == 1
    roots = quadraticRanges(q(1:2)/o(5),ray(:,1:2)/o(5));
    z = origin(3)+roots.*ray(:,3);
    roots(z<0 | z>2*o(7)) = inf;
    cap = (2*o(7)-origin(3))./ray(:,3);
    capXY = q(1:2)+cap.*ray(:,1:2);
    cap(cap<1e-6 | sum(capXY.^2,2)>o(5)^2) = inf;
    t = min([roots cap],[],2);
else
    c = cos(o(8)); a = sin(o(8));
    rotation = [c -a 0; a c 0; 0 0 1];
    radii = o(5:7);
    roots = quadraticRanges((q*rotation)./radii,(ray*rotation)./radii);
    roots(origin(3)+roots.*ray(:,3)<0) = inf;
    t = min(roots,[],2);
end
end

function roots = quadraticRanges(q,ray)
a = sum(ray.^2,2);
b = ray*q.';
c = sum(q.^2)-1;
delta = b.^2-a*c;
root = sqrt(max(delta,0));
roots = [-b-root -b+root]./a;
roots(roots<1e-6 | ~isfinite(roots) | delta<0 | a<1e-12) = inf;
end

function [f,h] = scanFigure(s,scan,interactive)
bg = [1 1 1];
ink = [0.13 0.19 0.24];
soft = [0.44 0.50 0.55];
route = [0.85 0.21 0.32];
reference = [0.06 0.48 0.76];
speedOnly = [0.02 0.54 0.46];
hasDetail = ~interactive && isfield(s,'detail');
f = figure('Name',['Simulated LiDAR mapping | ' s.title],'NumberTitle','off','Color',bg, ...
    'Units','inches','Position',[0.2 0.2 12.6 6.6], ...
    'DefaultAxesFontName','Arial','DefaultTextFontName','Arial');
if interactive
    label(f,[0.052 0.922 0.60 0.033],'(a)  Accumulated 3D returns',13,ink,'bold');
    label(f,[0.727 0.922 0.24 0.033],'(b)  Plan view',13,ink,'bold');
    position3 = [0.057 0.185 0.565 0.680];
    position2 = [0.733 0.185 0.197 0.680];
    barPosition = [0.638 0.260 0.011 0.480];
    scanBg = bg;
    scanInk = soft;
    legendY = 0.148;
    legendX = [0.053 0.226 0.412];
else
    f.Visible = 'off';
    f.Position(3:4) = [11 5.35];
    position3 = [0.095 0.214 0.528 0.622];
    position2 = [0.741 0.167 0.220 0.707];
    barPosition = [0.663 0.303 0.012 0.399];
    scanBg = [0.11 0.14 0.19];
    scanInk = [0.87 0.90 0.93];
    reference = [0.10 0.54 0.81];
    legendY = 0.063;
    legendX = [0.375 0.527 0.681];
    panelPosition = [0.044 0.149 0.602 0.738];
    scanTitle = [0.044 0.918 0.61 0.036];
    scanTitleText = ['(a)  ' s.title];
    planTitle = [0.745 0.918 0.21 0.036];
    if hasDetail
        f.Position(3:4) = [13.25 5.30];
        position3 = [0.072 0.234 0.448 0.612];
        position2 = [0.598 0.194 0.130 0.660];
        barPosition = [0.550 0.319 0.009 0.366];
        panelPosition = [0.033 0.163 0.499 0.722];
        scanTitle = [0.033 0.918 0.54 0.036];
        scanTitleText = [scanTitleText ' | Simulated LiDAR'];
        planTitle = [0.594 0.918 0.145 0.036];
        legendX = [0.306 0.449 0.594];
        if isempty(s.speedOnly)
            legendX = [0.373 0.518];
        end
    end
    label(f,scanTitle,scanTitleText,12,ink,'bold');
    label(f,planTitle,'(b)  Plan view',12,ink,'bold');

    % Keep the dark surround inside the 3D panel on a white export canvas.
    panel = axes(f,'Position',panelPosition, ...
        'XLim',[0 1],'YLim',[0 1],'Visible','off','HandleVisibility','off');
    patch(panel,[0 1 1 0],[0 0 1 1],scanBg,'EdgeColor','none');
end

ax3 = axes(f,'Position',position3);
ax2 = axes(f,'Position',position2);
for ax = [ax3 ax2]
    hold(ax,'on');
    set(ax,'Color',bg,'XColor',soft,'YColor',soft,'ZColor',soft, ...
        'GridColor',[0.82 0.86 0.88],'GridAlpha',0.30,'FontSize',9.5, ...
        'TickDir','out','CLim',[0 2]);
    grid(ax,'on');
    colormap(ax,scanColors());
end
if ~interactive
    set(ax3,'Color','none','XColor',scanInk,'YColor',scanInk,'ZColor',scanInk, ...
        'GridColor',[0.48 0.56 0.64],'GridAlpha',0.25);
end
h.cloud = scatter3(ax3,nan,nan,nan,0.85,nan,'filled', ...
    'MarkerEdgeColor','none','MarkerFaceAlpha',0.74);
h.planCloud = scatter(ax2,nan,nan,2.1,nan,'filled', ...
    'MarkerEdgeColor','none','MarkerFaceAlpha',0.88);

if interactive
    h.path = plot3(ax3,nan,nan,nan,'Color',route,'LineWidth',2.0);
    h.planPath = plot(ax2,nan,nan,'Color',route,'LineWidth',2.0);
    h.beams = plot3(ax3,nan,nan,nan,'Color',[0.34 0.73 0.69],'LineWidth',0.55);
    h.vehicle = plot3(ax3,nan,nan,nan,'o','MarkerFaceColor',route, ...
        'MarkerEdgeColor',ink,'MarkerSize',7);
    h.planVehicle = plot(ax2,nan,nan,'o','MarkerFaceColor',route, ...
        'MarkerEdgeColor',ink,'MarkerSize',6);
else
    updateCloud(h,scan,numel(scan.batches));
    drawRoute(ax3,ax2,s.path,route,'-',2.1);
end
if ~isempty(s.reference)
    drawRoute(ax3,ax2,s.reference,reference,'--',1.25);
end
if ~isempty(s.speedOnly)
    drawRoute(ax3,ax2,s.speedOnly,speedOnly,':',1.55);
end
plot3(ax3,s.start(1),s.start(2),s.start(3),'o','Color',scanInk,'MarkerFaceColor',scanBg);
plot(ax2,s.start(2),s.start(1),'o','Color',ink,'MarkerFaceColor',bg);
plot3(ax3,s.goal(1),s.goal(2),s.goal(3),'p','MarkerSize',9,'Color',scanInk,'MarkerFaceColor',scanBg);
plot(ax2,s.goal(2),s.goal(1),'p','MarkerSize',8,'Color',ink,'MarkerFaceColor',bg);

set(ax3,'XLim',s.xLimits,'YLim',s.yLimits,'ZLim',[0 2.25],'ZTick',[0 1 2], ...
    'Box','off','SortMethod','depth');
xlabel(ax3,'x (m)'); ylabel(ax3,'y (m)'); zlabel(ax3,'z (m)');
daspect(ax3,[1 1 1]);
view(ax3,-27,39);
camproj(ax3,'orthographic');
set(ax2,'XLim',s.yLimits,'YLim',s.xLimits,'Box','on','Layer','top');
daspect(ax2,[1 1 1]);
xlabel(ax2,'y (m)'); ylabel(ax2,'x (m)');
cb = colorbar(ax3);
cb.Position = barPosition;
cb.Color = soft;
cb.Ticks = [0 0.5 1 1.5 2];
cb.Label.String = 'Return height (m)';
cb.Label.Color = soft;
if ~interactive
    ax3.Position = position3;
    cb.Ticks = [0 1 2];
    cb.Label.String = 'Height (m)';
end
if hasDetail
    drawScanDetail(f,ax2,s,[reference;speedOnly;route],ink,soft);
end

if isempty(s.reference)
    key(f,legendX(1),legendY,route,'-','Illustrative path',ink);
else
    key(f,legendX(1),legendY,route,'-','Proposed',ink);
    key(f,legendX(2),legendY,reference,'--','Fixed EGO',ink);
    if ~isempty(s.speedOnly)
        key(f,legendX(3),legendY,speedOnly,':','Speed-only',ink);
    end
end
end

function drawScanDetail(f,overview,s,colors,ink,soft)
d = s.detail;
xLimits = d.detail_x_limits;
yLimits = d.detail_y_limits;
rectangle(overview,'Position',[yLimits(1),xLimits(1),diff(yLimits),diff(xLimits)], ...
    'EdgeColor',[0.35 0.44 0.50],'LineStyle','--','LineWidth',1.0);
label(f,[0.762 0.918 0.229 0.036],['(c)  ' d.detail_title],12,ink,'bold');
label(f,[0.772 0.855 0.208 0.033],sprintf('x = %.2f m',d.station),9,soft,'normal');
ax = axes(f,'Position',[0.765 0.205 0.199 0.613]);
hold(ax,'on');
angle = linspace(0,2*pi,151);
for side = [-1 1]
    x = s.wallNodes(:,1).';
    inner = side*s.wallNodes(:,2).';
    outer = inner+side*s.wallThickness;
    patch(ax,[inner fliplr(outer)],[x fliplr(x)],[0.83 0.88 0.90], ...
        'EdgeColor',[0.58 0.66 0.70],'LineWidth',0.65);
end
for k = 1:size(s.obstacles,1)
    o = s.obstacles(k,:);
    u = o(5)*cos(angle);
    v = o(6)*sin(angle);
    x = o(2)+cos(o(8))*u-sin(o(8))*v;
    y = o(3)+sin(o(8))*u+cos(o(8))*v;
    if max(x)<xLimits(1) || min(x)>xLimits(2) || max(y)<yLimits(1) || min(y)>yLimits(2)
        continue
    end
    face = [0.83 0.88 0.90];
    edge = [0.56 0.65 0.69];
    if k == d.target
        face = [0.97 0.82 0.58];
        edge = [0.65 0.41 0.13];
    end
    patch(ax,y,x,face,'EdgeColor',edge,'LineWidth',0.9);
end
plot(ax,yLimits,[d.station d.station],':','Color',soft,'LineWidth',0.95);
paths = {s.reference,s.speedOnly,s.path};
styles = {'--',':','-'};
active = find(~cellfun(@isempty,paths));
drawOrder = intersect([3 1 2],active,'stable');
points = nan(3,3);
points(active,:) = d.local_points;
for k = drawOrder
    p = paths{k};
    plot(ax,p(:,2),p(:,1),'-','Color','w','LineWidth',3.1);
end
for k = drawOrder
    p = paths{k};
    plot(ax,p(:,2),p(:,1),styles{k},'Color',colors(k,:),'LineWidth',1.8);
end
for k = drawOrder
    q = points(k,:);
    plot(ax,q(2)+d.body_radius*cos(angle),q(1)+d.body_radius*sin(angle), ...
        styles{k},'Color',colors(k,:),'LineWidth',1.15);
end
set(ax,'XLim',yLimits,'YLim',xLimits,'XColor',soft,'YColor',soft,'Color','w', ...
    'Box','on','Layer','top','FontSize',9.5,'GridColor',[0.62 0.69 0.73], ...
    'GridAlpha',0.30,'TickDir','out');
daspect(ax,[1 1 1]);
grid(ax,'on');
xlabel(ax,'y (m)');
ylabel(ax,'x (m)');
end

function colors = scanColors()
anchors = [0.34 0.29 0.66; 0.16 0.38 0.92; 0.16 0.77 0.97; ...
    0.19 0.87 0.73; 0.60 0.91 0.41; 0.99 0.87 0.34];
colors = interp1(linspace(0,1,size(anchors,1)),anchors,linspace(0,1,256));
end

function drawRoute(ax3,ax2,path,color,style,width)
plot3(ax3,path(:,1),path(:,2),path(:,3),style,'Color',color,'LineWidth',width);
plot(ax2,path(:,2),path(:,1),style,'Color',color,'LineWidth',width);
end

function key(f,x,y,color,style,name,ink)
annotation(f,'line',x+[0 0.028],[y y], ...
    'Color',color,'LineStyle',style,'LineWidth',1.7);
label(f,[x+0.036 y-0.013 0.15 0.026],name,9.5,ink,'normal');
end

function h = label(f,position,name,pointSize,color,weight)
h = annotation(f,'textbox',position,'String',name,'FontName','Arial', ...
    'FontSize',pointSize,'Color',color,'FontWeight',weight, ...
    'Interpreter','none','VerticalAlignment','middle','EdgeColor','none','Margin',0);
end

function updateCloud(h,scan,frame)
p = scan.points(scan.frames<=frame,:);
set(h.cloud,'XData',p(:,1),'YData',p(:,2),'ZData',p(:,3),'CData',p(:,3));
q = p(p(:,3)>0.10,:);
set(h.planCloud,'XData',q(:,2),'YData',q(:,1),'CData',q(:,3));
end

function addPlayback(f,h,scan,s)
frame = 1;
last = numel(scan.batches);
controls = {'Parent',f,'Units','normalized','FontName','Arial','FontSize',9};
button = uicontrol(controls{:},'Style','togglebutton','String','Play', ...
    'Position',[0.052 0.032 0.078 0.042],'Callback',@toggle);
slider = uicontrol(controls{:},'Style','slider','Min',1,'Max',last,'Value',1, ...
    'Position',[0.145 0.033 0.52 0.039], ...
    'SliderStep',[1 min(10,last-1)]/(last-1),'Callback',@seek);
status = label(f,[0.69 0.031 0.27 0.042],'',9.5,[0.13 0.19 0.24],'normal');
ticker = timer('ExecutionMode','fixedSpacing','Period',0.085,'TimerFcn',@advance);
f.CloseRequestFcn = @closeDemo;
showFrame(1);

    function toggle(~,~)
        if button.Value
            if frame==last
                showFrame(1);
            end
            button.String = 'Pause';
            start(ticker);
        else
            stop(ticker);
            button.String = 'Play';
        end
    end

    function seek(~,~)
        showFrame(round(slider.Value));
    end

    function advance(~,~)
        showFrame(min(frame+1,last));
        if frame==last
            stop(ticker);
            set(button,'Value',0,'String','Replay');
        end
    end

    function showFrame(next)
        frame = next;
        slider.Value = frame;
        updateCloud(h,scan,frame);
        path = scan.origins(1:frame,:);
        p = path(end,:);
        set(h.path,'XData',path(:,1),'YData',path(:,2),'ZData',path(:,3));
        set(h.planPath,'XData',path(:,2),'YData',path(:,1));
        set(h.vehicle,'XData',p(1),'YData',p(2),'ZData',p(3));
        set(h.planVehicle,'XData',p(2),'YData',p(1));
        q = scan.batches{frame};
        visible = q(:,1)>=s.xLimits(1) & q(:,1)<=s.xLimits(2) ...
            & q(:,2)>=s.yLimits(1) & q(:,2)<=s.yLimits(2) & q(:,3)>0.1;
        q = q(visible,:);
        q = q(1:max(1,ceil(size(q,1)/18)):end,:);
        segments = permute(cat(3,repmat(p,size(q,1),1),q,nan(size(q))),[3 1 2]);
        segments = reshape(segments,[],3);
        set(h.beams,'XData',segments(:,1),'YData',segments(:,2),'ZData',segments(:,3));
        status.String = sprintf('x = %+.1f m   |   %d mapped points',p(1),nnz(scan.frames<=frame));
    end

    function closeDemo(~,~)
        stop(ticker);
        delete(ticker);
        delete(f);
    end
end
