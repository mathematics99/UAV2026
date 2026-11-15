function render_scan_13(withSpeedOnly)
% Export the Scene 13 simulated LiDAR map with its rear-forest detail.
% Run render_scan_13(true) to export the three-method comparison separately.
arguments
    withSpeedOnly (1,1) logical = false
end
folder = fileparts(mfilename('fullpath'));
if withSpeedOnly
    data = load(fullfile(folder,'scan_13_speed_only_data.mat'));
    render_scan_maps(folder,'scan',data.case13,'_with_speed_only');
else
    data = load(fullfile(folder,'scan_13_data.mat'));
    render_scan_maps(folder,'scan',data.case13);
end
end
