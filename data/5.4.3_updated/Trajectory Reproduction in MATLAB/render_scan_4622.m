function render_scan_4622(sceneId)
% Export simulated LiDAR maps and local trajectory views for Scenes 46 and 22.
% Run render_scan_4622 for both scenes or render_scan_4622(46) for one scene.
folder = fileparts(mfilename('fullpath'));
data = load(fullfile(folder,'scan_4622_data.mat'));
scenes = [data.case46 data.case22];
if nargin > 0
    validateattributes(sceneId,{'numeric'},{'scalar','integer','finite'});
    scenes = scenes([scenes.scene_id] == sceneId);
    assert(~isempty(scenes),'Choose Scene 46 or Scene 22.');
end
render_scan_maps(folder,'scan',scenes);
end
