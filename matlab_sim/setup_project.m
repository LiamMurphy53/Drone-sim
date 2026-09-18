%% Add the project source folders to the MATLAB path
% Run this once after opening MATLAB in the project folder. The top-level run
% scripts also call it automatically.

projectRoot = fileparts(mfilename('fullpath'));

addpath(fullfile(projectRoot, 'src', 'core'));
addpath(fullfile(projectRoot, 'src', 'control'));
addpath(fullfile(projectRoot, 'src', 'visualization'));
