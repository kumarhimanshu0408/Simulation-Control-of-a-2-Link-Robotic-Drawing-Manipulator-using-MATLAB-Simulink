function main_gui_final2
% MAIN_GUI_FINAL2
% 2-link drawing manipulator GUI with robust SVG parsing, parameterized sampling,
% faster animation, Simulink export, and improved UX.
%
% Save this file as main_gui_final2.m and run:
%   clear main_gui_final2
%   main_gui_final2

%% ---- Default params ----
params.L1 = 1.0;
params.L2 = 1.0;
params.N  = 2000;            % default resampled points
params.curveSamples = 40;    % default samples per curve segment
params.totalTime = 8;        % seconds for full motion
params.elbow = 'down';
params.initX = 0.0;          % initial end-effector X (user-controlled)
params.initY = 0.0;          % initial end-effector Y

%% ---- Create UI ----
hFig = figure('Name','2-Link Drawing Manipulator (Final v2)','NumberTitle','off','Position',[150 80 1200 680]);

% Workspace axes
ax = axes('Parent',hFig,'Position',[0.04 0.12 0.58 0.83]);
axis(ax,[-(params.L1+params.L2) (params.L1+params.L2) -(params.L1+params.L2) (params.L1+params.L2)]);
axis(ax,'equal'); grid(ax,'on'); title(ax,'Workspace');

% Control panel
panel = uipanel('Parent',hFig,'Title','Controls','FontSize',10,'Position',[0.65 0.05 0.32 0.9]);

% --- Inputs ---
uicontrol(panel,'Style','text','Position',[10 520 120 20],'String','Arm L1','HorizontalAlignment','left','FontWeight','bold');
handles.hL1 = uicontrol(panel,'Style','edit','Position',[140 520 120 26],'String',num2str(params.L1));

uicontrol(panel,'Style','text','Position',[10 485 120 20],'String','Arm L2','HorizontalAlignment','left','FontWeight','bold');
handles.hL2 = uicontrol(panel,'Style','edit','Position',[140 485 120 26],'String',num2str(params.L2));

uicontrol(panel,'Style','text','Position',[10 450 120 20],'String','Trajectory Type','HorizontalAlignment','left','FontWeight','bold');
handles.hTraj = uicontrol(panel,'Style','popupmenu','Position',[140 450 140 26],'String',{'Circle','Line','Square','Custom','SVG'},'Value',1);

% Initial position (single parameter replaced Param1/Param2)
uicontrol(panel,'Style','text','Position',[10 415 120 20],'String','Initial X (m)','HorizontalAlignment','left');
handles.hInitX = uicontrol(panel,'Style','edit','Position',[140 415 60 26],'String',num2str(params.initX));
uicontrol(panel,'Style','text','Position',[210 415 40 20],'String','Y','HorizontalAlignment','left');
handles.hInitY = uicontrol(panel,'Style','edit','Position',[240 415 60 26],'String',num2str(params.initY));

% Sampling controls
uicontrol(panel,'Style','text','Position',[10 375 120 20],'String','Curve Samples','HorizontalAlignment','left');
handles.hCurveSamples = uicontrol(panel,'Style','edit','Position',[140 375 120 26],'String',num2str(params.curveSamples));

uicontrol(panel,'Style','text','Position',[10 340 120 20],'String','Resample N','HorizontalAlignment','left');
handles.hN = uicontrol(panel,'Style','edit','Position',[140 340 120 26],'String',num2str(params.N));

% Total time control (animation speed)
uicontrol(panel,'Style','text','Position',[10 305 120 20],'String','Total Time (s)','HorizontalAlignment','left');
handles.hTotalTime = uicontrol(panel,'Style','edit','Position',[140 305 120 26],'String',num2str(params.totalTime));

% Elbow choice
uicontrol(panel,'Style','text','Position',[10 265 120 20],'String','Elbow Choice','HorizontalAlignment','left');
handles.hElbow = uicontrol(panel,'Style','popupmenu','Position',[140 265 120 26],'String',{'down','up'},'Value',1);

% Guidance box
uicontrol(panel,'Style','text','Position',[10 220 260 30],'String','Tip: change Initial X,Y so the drawing lies inside robot reach.','HorizontalAlignment','left','ForegroundColor',[0 0.5 0]);
handles.hGuidance = uicontrol(panel,'Style','edit','Position',[10 175 260 40],'Max',2,'Min',0,'Enable','inactive','BackgroundColor',[0.95 0.98 1],...
    'String', {'GUIDANCE: Use Initial X,Y to translate the loaded SVG path into the robot workspace.'; ...
               'If trajectory appears incomplete, increase Curve Samples or Resample N.'}, 'HorizontalAlignment','left');

% Buttons
uicontrol(panel,'Style','pushbutton','Position',[10 120 120 36],'String','Generate Trajectory','Callback',@onGenerate);
uicontrol(panel,'Style','pushbutton','Position',[150 120 120 36],'String','Compute IK','Callback',@onComputeIK);
uicontrol(panel,'Style','pushbutton','Position',[10 70 120 36],'String','Animate','Callback',@onAnimate);
uicontrol(panel,'Style','pushbutton','Position',[150 70 120 36],'String','Move on →','Callback',@onMoveOn);

uicontrol(panel,'Style','pushbutton','Position',[10 20 120 36],'String','Reset','Callback',@onReset);
uicontrol(panel,'Style','pushbutton','Position',[150 20 120 36],'String','Upload SVG Path','BackgroundColor',[0.9 1 0.9],'Callback',@onUploadSVG);

% store
data.ax = ax; data.params = params; data.handles = handles;
guidata(hFig,data);

%% ---- CALLBACKS ----

    function onGenerate(~,~)
        % Generate trajectory from selected type or use stored SVG raw path
        d = guidata(hFig);
        % update local params from GUI
        d.params.L1 = str2double(get(d.handles.hL1,'String'));
        d.params.L2 = str2double(get(d.handles.hL2,'String'));
        cs = max(4, round(str2double(get(d.handles.hCurveSamples,'String'))));
        N = max(10, round(str2double(get(d.handles.hN,'String'))));
        initX = str2double(get(d.handles.hInitX,'String')); initY = str2double(get(d.handles.hInitY,'String'));
        trajStr = get(d.handles.hTraj,'String'); trajType = trajStr{get(d.handles.hTraj,'Value')};
        if strcmpi(trajType,'SVG')
            if ~isfield(d,'traj_raw') || isempty(d.traj_raw)
                errordlg('Upload SVG first.','No SVG'); return;
            end
            [x_t,y_t] = translate_and_scale_to_init(d.traj_raw.x, d.traj_raw.y, initX, initY, d.params.L1, d.params.L2);
            [x_res,y_res] = resample_and_smooth_path_param(x_t, y_t, N, cs);
        else
            [x_res,y_res] = generate_builtin(trajType, N);
            x_res = x_res + initX; y_res = y_res + initY;
        end
        d.traj.x = x_res(:)'; d.traj.y = y_res(:)'; guidata(hFig,d);
        cla(d.ax); hold(d.ax,'on');
        plot(d.ax,d.traj.x,d.traj.y,'-','LineWidth',2,'Color',[0 0 0.8]);
        plot(d.ax,0,0,'ko','MarkerFaceColor','k'); title(d.ax,'Trajectory Generated');
        axis(d.ax,[-(d.params.L1+d.params.L2) (d.params.L1+d.params.L2) -(d.params.L1+d.params.L2) (d.params.L1+d.params.L2)]);
        axis(d.ax,'equal'); grid(d.ax,'on'); hold(d.ax,'off');
    end

    function onComputeIK(~,~)
        d = guidata(hFig);
        % --- FIX: Read elbow selection from GUI so IK uses chosen elbow ---
        elbowStr = get(d.handles.hElbow,'String');
        d.params.elbow = elbowStr{get(d.handles.hElbow,'Value')};
        if ~isfield(d,'traj') || isempty(d.traj.x)
            errordlg('Generate trajectory first.','No Trajectory'); return;
        end
        L1 = d.params.L1; L2 = d.params.L2; elbow = d.params.elbow;
        X = d.traj.x; Y = d.traj.y; n = numel(X);
        th1 = NaN(1,n); th2 = NaN(1,n); okmask = false(1,n);
        for k = 1:n
            [t1,t2,ok] = inverse_kinematics(X(k),Y(k),L1,L2,elbow);
            if ok, th1(k)=t1; th2(k)=t2; okmask(k)=true; end
        end
        if ~any(okmask)
            errordlg('No reachable points with current L1/L2/position. Try changing Initial X,Y or arm lengths.','Unreachable');
            return;
        end
        % fill gaps, unwrap, smooth
        th1f = fill_and_interp_nans(th1);
        th2f = fill_and_interp_nans(th2);
        th1u = unwrap(th1f); th2u = unwrap(th2f);
        [t1_s,t2_s,~] = quintic_bspline_joint_smooth(th1u,th2u,d.params.N);
        % fallback if any NaN
        if any(isnan(t1_s)) || any(isnan(t2_s))
            valid = find(okmask);
            tq = linspace(1,n,d.params.N);
            t1_s = interp1(valid, th1u(valid), tq, 'pchip', 'extrap');
            t2_s = interp1(valid, th2u(valid), tq, 'pchip', 'extrap');
        end
        d.joint.theta1 = t1_s(:)'; d.joint.theta2 = t2_s(:)'; d.joint.ok = okmask;
        guidata(hFig,d);
        % forward-kin verify and plot small FK track on top of trajectory
        try
            [~,~,x_fk,y_fk] = forward_kinematics_traj(th1,th2,L1,L2);
            hold(d.ax,'on'); plot(d.ax,x_fk,y_fk,'r--','LineWidth',1.2); hold(d.ax,'off');
        catch
            % ignore FK plot errors
        end
        % show first pose
        update_links_plot(d,1);
        msgbox('IK computed and smoothed. You can now Animate or Move on →','IK Done');
    end

    function onAnimate(~,~)
        % Robust animate: recreates status text if missing; regulates realtime
        d = guidata(hFig);
        if ~isfield(d,'joint'), errordlg('Compute IK first.','No IK'); return; end

        N = numel(d.joint.theta1);
        if N == 0, errordlg('Empty joint trajectory.','Error'); return; end

        % total time and timestep
        T = max(0.001, d.params.totalTime);
        dt = T / max(1, N-1);

        % check axes
        if ~isfield(d,'ax') || ~ishandle(d.ax)
            errordlg('Axes lost. Please restart GUI.','Axes error'); return;
        end

        % obtain/create a single persistent text object (tag = 'statusText')
        htxt = [];
        try
            htxt = findobj(d.ax, '-regexp', 'Tag', '^statusText$');
            if isempty(htxt) || ~all(ishandle(htxt))
                % create new; place relative to axes limits
                lims = axis(d.ax);
                x_pos = lims(1) + 0.02*(lims(2)-lims(1));
                y_pos = lims(4) - 0.02*(lims(4)-lims(3));
                htxt = text(d.ax, x_pos, y_pos, '', 'FontSize',10, 'BackgroundColor','w', ...
                    'VerticalAlignment','top', 'EdgeColor',[0.8 0.8 0.8], 'Tag','statusText');
            else
                htxt = htxt(1); % take first if many
            end
        catch
            % as last resort create a new text on gca
            try
                htxt = text(d.ax, -(d.params.L1+d.params.L2)*0.9, (d.params.L1+d.params.L2)*0.9, '', ...
                    'FontSize',10,'BackgroundColor','w','VerticalAlignment','top','Tag','statusText');
            catch
                htxt = []; % we'll tolerate absence
            end
        end

        % store the text handle in guidata so future runs can find it quickly
        d.statusTextHandle = htxt; guidata(hFig,d);

        tStart = tic;
        for i = 1:N
            % sanity checks each iteration (handle or data might be invalidated)
            if ~isfield(d,'joint') || ~isreal(d.joint.theta1), break; end
            if isnan(d.joint.theta1(i)), continue; end

            % update plotting
            try
                update_links_plot(d, i);
            catch
                % if plotting fails, stop gracefully
                warning('update_links_plot failed at index %d; stopping animation.', i);
                break;
            end

            % update / recreate status text safely
            try
                if isempty(htxt) || ~ishandle(htxt)
                    % try to recreate (axes may have changed)
                    lims = axis(d.ax);
                    x_pos = lims(1) + 0.02*(lims(2)-lims(1));
                    y_pos = lims(4) - 0.02*(lims(4)-lims(3));
                    htxt = text(d.ax, x_pos, y_pos, '', 'FontSize',10, 'BackgroundColor','w', ...
                        'VerticalAlignment','top', 'EdgeColor',[0.8 0.8 0.8], 'Tag','statusText');
                    % store again
                    d.statusTextHandle = htxt; guidata(hFig,d);
                end
                if ~isempty(htxt) && ishandle(htxt)
                    set(htxt,'String', sprintf('θ1 = %.1f°\nθ2 = %.1f°', rad2deg(d.joint.theta1(i)), rad2deg(d.joint.theta2(i))));
                end
            catch
                % ignore set errors (object may have been deleted between check and set)
            end

            % regulate timing to avoid pauses/jitter
            elapsed = toc(tStart);
            target = (i-1) * dt;
            pause_time = target - elapsed;
            if pause_time > 0
                % small cap to avoid long sleeps if GUI gets blocked
                pause(min(pause_time, 0.05));
            else
                % running late: keep drawing but don't pause
                drawnow limitrate;
            end
        end
    end

        function onMoveOn(~,~)
        % Export motion references to base workspace for Simulink AND show compact plots
        d = guidata(hFig);
        if ~isfield(d,'joint') || isempty(d.joint) || isempty(d.joint.theta1)
            errordlg('Compute IK first.','No Data'); return;
        end

        % Ensure params updated from GUI (in case user changed total time)
        d.params.totalTime = max(0.001, str2double(get(d.handles.hTotalTime,'String')));
        guidata(hFig,d);

        N = numel(d.joint.theta1);
        T = max(0.001,d.params.totalTime);
        tvec = linspace(0,T,N)';

        theta1_ref = [tvec, d.joint.theta1(:)];
        theta2_ref = [tvec, d.joint.theta2(:)];

        % compute end-effector XY trajectory to export to Simulink XY graph
        [x1,y1,x2,y2] = forward_kinematics_traj(d.joint.theta1,d.joint.theta2,d.params.L1,d.params.L2);
        ee_ref = [tvec, x2(:), y2(:)]; % time,x,y

        % assign arrays
        assignin('base','theta1_ref',theta1_ref);
        assignin('base','theta2_ref',theta2_ref);
        assignin('base','ee_ref',ee_ref);
        assignin('base','L1_sim',d.params.L1); assignin('base','L2_sim',d.params.L2);
        assignin('base','Kp',10); assignin('base','Kd',2);

        % also export timeseries objects (handy for Simulink)
        try
            ts_theta1 = timeseries(theta1_ref(:,2), theta1_ref(:,1));
            ts_theta2 = timeseries(theta2_ref(:,2), theta2_ref(:,1));
            ts_ee = timeseries(ee_ref(:,2:3), ee_ref(:,1)); % data = [x y]
            assignin('base','ts_theta1',ts_theta1);
            assignin('base','ts_theta2',ts_theta2);
            assignin('base','ts_ee',ts_ee);
        catch
            % ignore if timeseries toolbox not available
        end

        % Notify user
        msgbox('Data exported to base workspace: theta1_ref, theta2_ref, ee_ref. Plotting compact diagnostics...','Exported');

        % Call compact plotting helper
        try
            control_analysis_exhibit(d);
        catch ME
            warning('control_analysis_compact failed: %s', ME.message);
        end
    end


    function onReset(~,~)
        d = guidata(hFig);
        cla(d.ax); grid(d.ax,'on'); title(d.ax,'Workspace');
        data = struct('ax',d.ax,'params',params,'handles',d.handles);
        guidata(hFig,data);
        set(d.handles.hL1,'String',num2str(params.L1)); set(d.handles.hL2,'String',num2str(params.L2));
        set(d.handles.hCurveSamples,'String',num2str(params.curveSamples)); set(d.handles.hN,'String',num2str(params.N));
        set(d.handles.hTotalTime,'String',num2str(params.totalTime));
        set(d.handles.hInitX,'String',num2str(params.initX)); set(d.handles.hInitY,'String',num2str(params.initY));
        set(d.handles.hTraj,'Value',1);
    end

    function onUploadSVG(~,~)
        d = guidata(hFig);
        [file,path] = uigetfile({'*.svg;*.SVG','SVG files (*.svg)'}, 'Select an SVG file');
        if isequal(file,0), return; end
        fullPath = fullfile(path,file);
        fprintf('Loading SVG: %s\n', fullPath);
        try
            % read sampling parameters from GUI (use current settings)
            cs = max(4, round(str2double(get(d.handles.hCurveSamples,'String'))));
            N = max(10, round(str2double(get(d.handles.hN,'String'))));
            % parse raw paths (returns centered & scaled coords)
            [xraw, yraw] = parseSVGPath_enhanced_fixed(fullPath, cs);
            if isempty(xraw)
                errordlg('No path data parsed from SVG.','SVG Error'); return;
            end
            % Store raw normalized version
            d.traj_raw.x = xraw(:)'; d.traj_raw.y = yraw(:)';
            % Immediately prepare a resampled `d.traj` using current initial X/Y
            initX = str2double(get(d.handles.hInitX,'String')); initY = str2double(get(d.handles.hInitY,'String'));
            [x_trans,y_trans] = translate_and_scale_to_init(d.traj_raw.x, d.traj_raw.y, initX, initY, d.params.L1, d.params.L2);
            [x_res,y_res] = resample_and_smooth_path_param(x_trans, y_trans, N, cs);
            d.traj.x = x_res(:)'; d.traj.y = y_res(:)';
            % store back and update UI
            guidata(hFig,d);
            cla(d.ax); hold(d.ax,'on');
            plot(d.ax, d.traj.x, d.traj.y, '-', 'LineWidth', 1.5, 'Color', [0.6 0 0.6]);
            plot(d.ax, 0, 0, 'ko','MarkerFaceColor','k');
            title(d.ax,'SVG Loaded and Resampled');
            axis(d.ax,'equal'); grid(d.ax,'on'); hold(d.ax,'off');
            helpdlg('SVG loaded and resampled. Now click Compute IK or Generate Trajectory if you change params.', 'SVG loaded');
        catch ME
            errordlg(['Failed to process SVG: ' ME.message], 'SVG Error');
            fprintf(2,'SVG parse error: %s\n', ME.message);
            if isfield(ME,'stack'), for s=1:numel(ME.stack), fprintf(2,'  In %s at line %d\n', ME.stack(s).name, ME.stack(s).line); end; end
        end
    end

%% ---- Nested IK (kept nested for simplicity) ----
    function [theta1, theta2, ok] = inverse_kinematics(px,py,L1,L2,elbow)
        r2 = px^2 + py^2; r = sqrt(r2);
        if r > (L1+L2) || r < abs(L1-L2)
            theta1 = NaN; theta2 = NaN; ok = false; return;
        end
        cos_th2 = (r2 - L1^2 - L2^2)/(2*L1*L2); cos_th2 = max(-1,min(1,cos_th2));
        if strcmpi(elbow,'down')
            theta2 = atan2(-sqrt(1-cos_th2^2), cos_th2);
        else
            theta2 = atan2( sqrt(1-cos_th2^2), cos_th2);
        end
        k1 = L1 + L2*cos(theta2); k2 = L2*sin(theta2);
        theta1 = atan2(py,px) - atan2(k2,k1);
        ok = true;
    end

end % ---- END OF MAIN FUNCTION ----


%% ---------------- Subfunctions ----------------
% Place all helper functions after the main function (not nested) so they are visible.

function control_analysis_exhibit(d)
% control_analysis_exhibit(d)
% Produces 6 exhibition-friendly plots:
% 1) Initial manipulator pose, 2) End-effector desired vs actual (XY),
% 3) Joint angles (deg), 4) Joint velocities (deg/s),
% 5) End-effector tracking error (m), 6) PD-like torques (arb).
%
% Call from onMoveOn after d.joint is ready.

if ~isfield(d,'joint') || isempty(d.joint) || isempty(d.joint.theta1)
    errordlg('No joint data found. Compute IK first.','No Data'); return;
end

% data
theta1 = d.joint.theta1(:);
theta2 = d.joint.theta2(:);
N = numel(theta1);
T = max(0.001, d.params.totalTime);
t = linspace(0, T, N)';

% forward kinematics actual (from smoothed joint refs)
[x1a,y1a,x2a,y2a] = forward_kinematics_traj(theta1, theta2, d.params.L1, d.params.L2);

% desired trajectory (if available)
if isfield(d,'traj') && ~isempty(d.traj.x)
    xd = d.traj.x(:); yd = d.traj.y(:);
    % if number of desired points differs from N, resample to match time vector
    if numel(xd) ~= N
        xd = interp1(linspace(0,1,numel(xd)), xd, linspace(0,1,N), 'pchip');
        yd = interp1(linspace(0,1,numel(yd)), yd, linspace(0,1,N), 'pchip');
    end
else
    xd = nan(N,1); yd = nan(N,1);
end

% velocities (finite difference) -> convert to deg/s for readability
dt = t(2) - t(1);
omega1 = [0; diff(theta1)] / dt; % rad/s
omega2 = [0; diff(theta2)] / dt; % rad/s

% tracking error (end-effector)
ee_err = sqrt( (x2a - xd).^2 + (y2a - yd).^2 );

% PD-like torques (use same Kp/Kd as GUI uses)
Kp = 10; Kd = 2;
e1 = theta1 - theta1(end); e2 = theta2 - theta2(end);
dq1 = [0; diff(theta1)]; dq2 = [0; diff(theta2)];
tau1 = Kp*e1 + Kd*(-dq1);
tau2 = Kp*e2 + Kd*(-dq2);

% Create figure
hf = figure('Name','Exhibit Diagnostics','NumberTitle','off','Position',[200 60 1200 760]);
tiledlayout(3,2,'TileSpacing','compact','Padding','compact');

% 1) Initial pose (large)
nexttile([1 1]);
plot([0, d.params.L1*cos(theta1(1)), d.params.L1*cos(theta1(1)) + d.params.L2*cos(theta1(1)+theta2(1))], ...
     [0, d.params.L1*sin(theta1(1)), d.params.L1*sin(theta1(1)) + d.params.L2*sin(theta1(1)+theta2(1))], '-o','LineWidth',2);
axis equal; grid on; title('Initial Pose'); xlabel('X (m)'); ylabel('Y (m)');

% 2) End-effector desired vs actual (XY)
nexttile([1 1]);
hold on;
if ~all(isnan(xd))
    plot(xd, yd, 'k--', 'LineWidth',1.5); % desired
end
plot(x2a, y2a, 'r-', 'LineWidth',1.5); % actual
legend('Desired','Actual','Location','best'); axis equal; grid on;
title('End-effector Path'); xlabel('X (m)'); ylabel('Y (m)');
hold off;

% 3) Joint angles (deg)
nexttile([1 1]);
plot(t, rad2deg(theta1), 'LineWidth',1.4); hold on;
plot(t, rad2deg(theta2), 'LineWidth',1.4); hold off;
legend('\theta_1','\theta_2','Location','best'); xlabel('Time (s)'); ylabel('Angle (°)'); grid on;
title('Joint Angles');

% 4) Joint velocities (deg/s)
nexttile([1 1]);
plot(t, rad2deg(omega1), 'LineWidth',1.2); hold on;
plot(t, rad2deg(omega2), 'LineWidth',1.2); hold off;
legend('\omega_1','\omega_2','Location','best'); xlabel('Time (s)'); ylabel('Velocity (°/s)'); grid on;
title('Joint Velocities');

% 5) End-effector tracking error
nexttile([1 1]);
plot(t, ee_err, 'LineWidth',1.4);
xlabel('Time (s)'); ylabel('Error (m)'); grid on; title('End-effector Tracking Error');
% show a small annotation with max / RMS error
max_err = max(ee_err);
rms_err = sqrt(mean(ee_err.^2));
text(0.02*T, 0.9*max(ee_err + eps), sprintf('Max=%.3f m  RMS=%.3f m', max_err, rms_err), 'FontSize',9, 'BackgroundColor','w');

% 6) PD-like torques
nexttile([1 1]);
plot(t, tau1, 'LineWidth',1.2); hold on;
plot(t, tau2, 'LineWidth',1.2); hold off;
legend('\tau_1','\tau_2','Location','best'); xlabel('Time (s)'); ylabel('Torque (arb)'); grid on; title('PD-like Torques');

% tidy
sgtitle('Exhibit Diagnostics: Path Tracking & Motion Profiles','FontSize',14);
end


function [xcol,ycol] = parseSVGPath_enhanced_fixed(svg_file, curve_resolution)
% Robust parser for common SVG commands (M L H V C S Q T A Z).
% Returns centered & scaled column vectors (normalized).
if nargin<2, curve_resolution=40; end
xcol=zeros(0,1); ycol=zeros(0,1);
try
    xmlDoc = xmlread(svg_file);
catch ME
    error('xmlread: %s', ME.message);
end

d_str = '';
% gather path, polyline, polygon
pathNodes = xmlDoc.getElementsByTagName('path');
for i=0:pathNodes.getLength-1
    s = char(pathNodes.item(i).getAttribute('d'));
    if ~isempty(s), d_str = [d_str ' ' s]; end
end
plNodes = xmlDoc.getElementsByTagName('polyline');
for i=0:plNodes.getLength-1
    pts = char(plNodes.item(i).getAttribute('points'));
    if ~isempty(pts), d_str = [d_str ' M ' pts ' Z ']; end
end
pgNodes = xmlDoc.getElementsByTagName('polygon');
for i=0:pgNodes.getLength-1
    pts = char(pgNodes.item(i).getAttribute('points'));
    if ~isempty(pts), d_str = [d_str ' M ' pts ' Z ']; end
end

d_str = strtrim(d_str);
if isempty(d_str), return; end

tokens_raw = regexp(d_str, '([MmLlHhVvCcSsQqTtAaZz])|(-?\d*\.?\d+(?:[eE][-+]?\d+)?)', 'tokens');
tokens = [tokens_raw{:}];

isCommand = @(tok) ~isempty(tok) && ischar(tok) && numel(tok)==1 && isletter(tok);
i = 1; curpos=[0,0]; startpos=[0,0]; last_ctrl=[];
while i <= numel(tokens)
    token = tokens{i};
    if isCommand(token)
        cmd = token; i=i+1;
    else
        if ~exist('cmd','var') || isempty(cmd), i=i+1; continue; end
    end
    switch cmd
        case {'M','m'}
            [vals,i] = read_numbers(tokens,i);
            if isempty(vals), continue; end
            pts = reshape(vals,2,[])';
            for k=1:size(pts,1)
                if strcmp(cmd,'m'), curpos = curpos + pts(k,:); else curpos = pts(k,:); end
                [xcol,ycol] = safe_append(xcol,ycol,curpos);
                if k==1, startpos = curpos; end
                last_ctrl=[];
            end
        case {'L','l'}
            [vals,i] = read_numbers(tokens,i);
            if isempty(vals), continue; end
            pts = reshape(vals,2,[])';
            for k=1:size(pts,1)
                if strcmp(cmd,'l'), curpos = curpos + pts(k,:); else curpos = pts(k,:); end
                [xcol,ycol] = safe_append(xcol,ycol,curpos); last_ctrl=[];
            end
        case {'H','h'}
            [vals,i] = read_numbers(tokens,i);
            for k=1:numel(vals)
                if strcmp(cmd,'h'), curpos(1)=curpos(1)+vals(k); else curpos(1)=vals(k); end
                [xcol,ycol]=safe_append(xcol,ycol,curpos); last_ctrl=[];
            end
        case {'V','v'}
            [vals,i] = read_numbers(tokens,i);
            for k=1:numel(vals)
                if strcmp(cmd,'v'), curpos(2)=curpos(2)+vals(k); else curpos(2)=vals(k); end
                [xcol,ycol]=safe_append(xcol,ycol,curpos); last_ctrl=[];
            end
        case {'C','c'}
            [vals,i] = read_numbers(tokens,i);
            if isempty(vals), continue; end
            pts = reshape(vals,2,[])';
            if mod(size(pts,1),3)~=0, pts = pts(1:floor(size(pts,1)/3)*3,:); end
            for s=1:3:size(pts,1)
                c1=pts(s,:); c2=pts(s+1,:); ep=pts(s+2,:);
                if strcmp(cmd,'c'), c1=curpos+c1; c2=curpos+c2; ep=curpos+ep; end
                seg = sampleCubic(curpos,c1,c2,ep,curve_resolution);
                [xcol,ycol] = safe_append(xcol,ycol,seg(2:end,:));
                curpos = ep; last_ctrl=c2;
            end
        case {'S','s'}
            [vals,i] = read_numbers(tokens,i);
            if isempty(vals), continue; end
            pts=reshape(vals,2,[])';
            if mod(size(pts,1),2)~=0, pts=pts(1:floor(size(pts,1)/2)*2,:); end
            for s=1:2:size(pts,1)
                c2=pts(s,:); ep=pts(s+1,:);
                if strcmp(cmd,'s'), c2=curpos+c2; ep=curpos+ep; end
                if isempty(last_ctrl), c1=curpos; else c1 = curpos + (curpos - last_ctrl); end
                seg = sampleCubic(curpos,c1,c2,ep,curve_resolution);
                [xcol,ycol] = safe_append(xcol,ycol,seg(2:end,:)); curpos=ep; last_ctrl=c2;
            end
        case {'Q','q'}
            [vals,i] = read_numbers(tokens,i);
            if isempty(vals), continue; end
            pts=reshape(vals,2,[])';
            if mod(size(pts,1),2)~=0, pts=pts(1:floor(size(pts,1)/2)*2,:); end
            for s=1:2:size(pts,1)
                c=pts(s,:); ep=pts(s+1,:);
                if strcmp(cmd,'q'), c=curpos+c; ep=curpos+ep; end
                seg = sampleQuadratic(curpos,c,ep,curve_resolution);
                [xcol,ycol] = safe_append(xcol,ycol,seg(2:end,:)); curpos=ep; last_ctrl=c;
            end
        case {'T','t'}
            [vals,i] = read_numbers(tokens,i);
            if isempty(vals), continue; end
            pts=reshape(vals,2,[])';
            for s=1:size(pts,1)
                ep=pts(s,:); if strcmp(cmd,'t'), ep=curpos+ep; end
                if isempty(last_ctrl), c=curpos; else c=curpos + (curpos - last_ctrl); end
                seg = sampleQuadratic(curpos,c,ep,curve_resolution);
                [xcol,ycol] = safe_append(xcol,ycol,seg(2:end,:)); curpos=ep; last_ctrl=c;
            end
        case {'A','a'}
            [vals,i] = read_numbers(tokens,i);
            if isempty(vals), continue; end
            if mod(numel(vals),7)~=0, vals = vals(1:floor(numel(vals)/7)*7); end
            vals = reshape(vals,7,[])';
            for s=1:size(vals,1)
                rx=vals(s,1); ry=vals(s,2); phi=vals(s,3);
                laf = vals(s,4)~=0; sf = vals(s,5)~=0;
                x2=vals(s,6); y2=vals(s,7);
                if strcmp(cmd,'a'), x2 = curpos(1)+x2; y2 = curpos(2)+y2; end
                seg = svg_arc_to_poly_fast(curpos,[x2,y2],rx,ry,phi,laf,sf,curve_resolution);
                [xcol,ycol] = safe_append(xcol,ycol,seg(2:end,:)); curpos=[x2,y2]; last_ctrl=[];
            end
        case {'Z','z'}
            if ~isempty(xcol), [xcol,ycol] = safe_append(xcol,ycol,startpos); curpos=startpos; end
            last_ctrl=[];
        otherwise
            % ignore unsupported commands
    end
end

% cleanup: remove NaNs, duplicates, center & scale
pts = [xcol,ycol]; pts(any(isnan(pts),2),:)=[]; if isempty(pts), xcol=zeros(0,1); ycol=zeros(0,1); return; end
if size(pts,1)>1
    dvec = sqrt(sum(diff(pts).^2,2)); keep = [true; dvec>eps]; pts = pts(keep,:);
end
xcol = pts(:,1) - mean(pts(:,1)); ycol = pts(:,2) - mean(pts(:,2));
mxd = max((max(xcol)-min(xcol)), (max(ycol)-min(ycol))); if mxd>0, xcol = xcol / mxd; ycol = ycol / mxd; end
end

function [xr, yr] = resample_and_smooth_path_param(x, y, N, curveSamples)
% Robust resample: remove duplicates, create cumulative distance, enforce unique tOrig,
% then pchip-resample and light smooth.
x = x(:)'; y = y(:)';
if numel(x) < 2
    xr = repmat(x(1),1,N); yr = repmat(y(1),1,N); return;
end

% Remove consecutive duplicate points (or almost-duplicates)
d2 = diff([x; y]') ;
dists = sqrt(sum(d2.^2,2));
keepMask = [true; dists > (eps*100)]; % tolerance bumped to avoid numerical duplicates
x = x(keepMask); y = y(keepMask);
if numel(x) < 2
    xr = repmat(x(1),1,N); yr = repmat(y(1),1,N); return;
end

% cumulative distance
seg = sqrt(diff(x).^2 + diff(y).^2);
cumd = [0, cumsum(seg)];
if cumd(end) == 0
    xr = repmat(x(1),1,N); yr = repmat(y(1),1,N); return;
end

% ensure unique parameterization
[tOrig, ia] = unique(cumd, 'stable');
x_u = x(ia); y_u = y(ia);

% If unique reduced to <2, fallback to simple linear segment
if numel(tOrig) < 2
    xr = linspace(x_u(1), x_u(end), N);
    yr = linspace(y_u(1), y_u(end), N);
    return;
end

tOrig = tOrig / tOrig(end);      % normalize 0..1
tNew = linspace(0,1,N);

% Use 'pchip' for shape preservation
xr = interp1(tOrig, x_u, tNew, 'pchip');
yr = interp1(tOrig, y_u, tNew, 'pchip');

% slight smoothing using moving median and mean to avoid artifacts for complex SVGs
w = max(3, round(N * 0.003));    % ~0.3% window
xr = movmedian(xr, w);
yr = movmedian(yr, w);
xr = movmean(xr, w);
yr = movmean(yr, w);

% final safety: ensure no NaN
if any(isnan(xr)), xr(isnan(xr)) = interp1(find(~isnan(xr)), xr(~isnan(xr)), find(isnan(xr)), 'pchip', 'extrap'); end
if any(isnan(yr)), yr(isnan(yr)) = interp1(find(~isnan(yr)), yr(~isnan(yr)), find(isnan(yr)), 'pchip', 'extrap'); end

xr = xr(:)'; yr = yr(:)';
end

function [x_out,y_out] = translate_and_scale_to_init(x_norm,y_norm,initX,initY,L1,L2)
% scale normalized path so that it fits within reach and translate to initial pos
x = x_norm(:)'; y = y_norm(:)';
extent = max(max(x)-min(x), max(y)-min(y));
if extent <= 0, scale = 0.5*min(L1+L2,1); else scale = 0.8*min(L1+L2,1)/extent; end
x_out = x * scale + initX; y_out = y * scale + initY;
end

function [x_out,y_out] = generate_builtin(type,N)
t = linspace(0,2*pi,N);
switch type
    case 'Circle'
        R = 0.5; x_out = R*cos(t); y_out = R*sin(t);
    case 'Line'
        x_out = linspace(-0.5,0.5,N); y_out = zeros(1,N);
    case 'Square'
        side = 1.0; pts=[-side/2 -side/2; side/2 -side/2; side/2 side/2; -side/2 side/2; -side/2 -side/2];
        Nside = ceil(N/4); x=[]; y=[];
        for i=1:4
            xs = linspace(pts(i,1), pts(i+1,1), Nside);
            ys = linspace(pts(i,2), pts(i+1,2), Nside);
            x = [x xs(1:end-1)]; y = [y ys(1:end-1)];
        end
        x = [x x(1)]; y = [y y(1)];
        dist = [0 cumsum(sqrt(diff(x).^2 + diff(y).^2))]; tOrig = dist/dist(end); tNew = linspace(0,1,N);
        x_out = interp1(tOrig, x, tNew, 'spline'); y_out = interp1(tOrig, y, tNew, 'spline');
    case 'Custom'
        title(gca,'Click points; double-click to finish'); [xc,yc] = getpts(gca);
        if numel(xc)<2, x_out=[]; y_out=[]; return; end
        tOrig = linspace(0,1,numel(xc)); tNew = linspace(0,1,N);
        x_out = interp1(tOrig, xc, tNew, 'spline'); y_out = interp1(tOrig, yc, tNew, 'spline');
    otherwise
        x_out = zeros(1,N); y_out = zeros(1,N);
end
end

function [vals, next_i] = read_numbers(tokens, i_start)
vals = []; i = i_start;
while i <= numel(tokens)
    tok = tokens{i};
    if iscell(tok), tok = tok{1}; end
    if ischar(tok) && numel(tok)==1 && isletter(tok), break; end
    num = str2double(tok);
    if isnan(num), break; end
    vals(end+1)=num; i=i+1;
end
next_i = i;
end

function [xout,yout] = safe_append(xin,yin,seg)
if isempty(seg), xout=xin; yout=yin; return; end
if iscell(seg), seg = cell2mat(seg); end
[r,c] = size(seg);
if r==1 && c==2
    seg_pts = seg;
elseif r>1 && c==2
    seg_pts = seg;
elseif r>1 && c>2 && mod(c,2)==0
    seg_pts = reshape(seg,2,[])';
else
    error('safe_append:badShape','Segment cannot be reshaped to Nx2');
end
if isempty(xin)
    xout = seg_pts(:,1); yout = seg_pts(:,2); return;
end
xout = [xin(:); seg_pts(:,1)]; yout = [yin(:); seg_pts(:,2)];
end

function pts = sampleCubic(p0,c1,c2,p3,res)
if nargin<5 || isempty(res), res = 40; end
t = linspace(0,1,res+1)'; pts=zeros(numel(t),2);
for k=1:numel(t)
    tt=t(k);
    pts(k,:) = (1-tt)^3*p0 + 3*(1-tt)^2*tt*c1 + 3*(1-tt)*tt^2*c2 + tt^3*p3;
end
end

function pts = sampleQuadratic(p0,c,p2,res)
if nargin<4 || isempty(res), res = 40; end
t = linspace(0,1,res+1)'; pts=zeros(numel(t),2);
for k=1:numel(t)
    tt = t(k);
    pts(k,:) = (1-tt)^2*p0 + 2*(1-tt)*tt*c + tt^2*p2;
end
end

function pts = svg_arc_to_poly_fast(p1,p2,rx,ry,phi_deg,laf,sf,res)
% Approximate an SVG elliptical arc by sampling points
if nargin<8, res=40; end
if rx==0 || ry==0 || all(p1==p2)
    pts = [linspace(p1(1),p2(1),res+1)', linspace(p1(2),p2(2),res+1)'];
    return;
end
phi = deg2rad(phi_deg);
dx = (p1(1)-p2(1))/2; dy = (p1(2)-p2(2))/2;
xp = cos(phi)*dx + sin(phi)*dy; yp = -sin(phi)*dx + cos(phi)*dy;
rx = abs(rx); ry = abs(ry);
lam = (xp^2)/(rx^2) + (yp^2)/(ry^2);
if lam > 1, k = sqrt(lam); rx = rx * k; ry = ry * k; end
num = rx^2*ry^2 - rx^2*yp^2 - ry^2*xp^2;
den = rx^2*yp^2 + ry^2*xp^2;
num = max(num,0);
if den == 0
    cxp = 0; cyp = 0;
else
    signTerm = 1; if laf == sf, signTerm = -1; end
    factor = 0; if den > 0, factor = signTerm * sqrt(num/den); end
    cxp = factor * (rx*yp/ry); cyp = factor * (-ry*xp/rx);
end
cx = cos(phi)*cxp - sin(phi)*cyp + (p1(1)+p2(1))/2;
cy = sin(phi)*cxp + cos(phi)*cyp + (p1(2)+p2(2))/2;
v1 = [(xp - cxp)/rx, (yp - cyp)/ry]; v2 = [(-xp - cxp)/rx, (-yp - cyp)/ry];
ang1 = atan2(v1(2),v1(1)); ang2 = atan2(v2(2),v2(1)); dang = ang2 - ang1;
if sf && (dang < 0), dang = dang + 2*pi;
elseif (~sf) && (dang > 0), dang = dang - 2*pi; end
theta = linspace(ang1, ang1 + dang, max(3,res+1));
X = rx * cos(theta); Y = ry * sin(theta);
pts = zeros(numel(theta),2);
for k=1:numel(theta)
    pts(k,1) = cos(phi)*X(k) - sin(phi)*Y(k) + cx;
    pts(k,2) = sin(phi)*X(k) + cos(phi)*Y(k) + cy;
end
end

function a = fill_and_interp_nans(a)
a = a(:)'; valid = ~isnan(a);
if sum(valid) < 2, a(~valid)=0; return; end
xi = 1:numel(a); xv=xi(valid); yv=a(valid);
a = interp1(xv,yv, xi, 'pchip', 'extrap');
end

function [t1_s,t2_s,t] = quintic_bspline_joint_smooth(th1,th2,Nout)
n = numel(th1);
if n < 4
    t = linspace(0,1,Nout);
    t1_s = interp1(1:n, th1, linspace(1,n,Nout), 'pchip', 'extrap');
    t2_s = interp1(1:n, th2, linspace(1,n,Nout), 'pchip', 'extrap');
    return;
end
try
    % Attempt B-spline smoothing (if spline toolbox available)
    knots = augknt(linspace(0,1,max(8,round(n/4))),6);
    sp1 = spap2(knots,6, linspace(0,1,n), th1);
    sp2 = spap2(knots,6, linspace(0,1,n), th2);
    t = linspace(0,1,Nout);
    t1_s = fnval(sp1,t); t2_s = fnval(sp2,t);
catch
    % fallback
    t = linspace(0,1,Nout);
    t1_s = spline(linspace(0,1,n), th1, t);
    t2_s = spline(linspace(0,1,n), th2, t);
end
end

function update_links_plot(d, idx)
% Defensive plot update for the manipulator at index idx
if ~isfield(d,'ax') || ~ishandle(d.ax)
    return;
end
if ~isfield(d,'joint') || idx < 1 || idx > numel(d.joint.theta1)
    return;
end
q1 = d.joint.theta1(idx); q2 = d.joint.theta2(idx);
L1 = d.params.L1; L2 = d.params.L2;
p0=[0 0]; p1=[L1*cos(q1), L1*sin(q1)]; p2=[p1(1)+L2*cos(q1+q2), p1(2)+L2*sin(q1+q2)];
cla(d.ax); hold(d.ax,'on');
theta = linspace(0,2*pi,180);
plot(d.ax,(L1+L2)*cos(theta),(L1+L2)*sin(theta),'--k','LineWidth',0.5);
if L1 > L2, plot(d.ax,(L1-L2)*cos(theta),(L1-L2)*sin(theta),'--k','LineWidth',0.5); end
if isfield(d,'traj') && ~isempty(d.traj.x), plot(d.ax,d.traj.x,d.traj.y,'--','Color',[0.7 0.7 0.7]); end
if isfield(d,'joint')
    try
        [~,~,x_fk,y_fk] = forward_kinematics_traj(d.joint.theta1,d.joint.theta2,L1,L2);
        plot(d.ax, x_fk(1:idx), y_fk(1:idx), 'r-','LineWidth',1.5);
    catch
        % ignore
    end
end
plot(d.ax,[p0(1) p1(1) p2(1)],[p0(2) p1(2) p2(2)],'o-','LineWidth',3,'MarkerSize',8);
plot(d.ax,p0(1),p0(2),'sk','MarkerFaceColor','k','MarkerSize',10);
plot(d.ax,p2(1),p2(2),'ro','MarkerFaceColor','r','MarkerSize',8);
axis(d.ax,[-(L1+L2) (L1+L2) -(L1+L2) (L1+L2)]);
axis(d.ax,'equal'); grid(d.ax,'on'); title(d.ax,'Workspace Animation'); hold(d.ax,'off');
end

function [x1,y1,x2,y2] = forward_kinematics_traj(th1,th2,L1,L2)
x1 = L1*cos(th1); y1 = L1*sin(th1);
x2 = x1 + L2*cos(th1+th2); y2 = y1 + L2*sin(th1+th2);
end