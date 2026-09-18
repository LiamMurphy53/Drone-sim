function droneInputs = loadDroneInputFile(inputFile)
%LOADDRONEINPUTFILE Run a drone-input script in an isolated workspace.
%
% droneInputs = loadDroneInputFile(inputFile)
%
% The selected .m script must create one scalar structure named
% droneInputs. Only that structure is returned; variables created by the
% script stay inside this function and do not enter the caller workspace.

    if isstring(inputFile)
        if ~isscalar(inputFile)
            error('loadDroneInputFile:InvalidFileName', ...
                'inputFile must be one text scalar.');
        end
        inputFile = char(inputFile);
    end
    if ~ischar(inputFile) || isempty(strtrim(inputFile)) || ...
            ~isrow(inputFile)
        error('loadDroneInputFile:InvalidFileName', ...
            'inputFile must be one nonempty character vector or string.');
    end

    inputFile = strtrim(inputFile);
    if ~isfile(inputFile)
        resolvedFile = which(inputFile);
        if isempty(resolvedFile)
            error('loadDroneInputFile:FileNotFound', ...
                'Drone input file not found: %s', inputFile);
        end
        inputFile = resolvedFile;
    end
    [~, ~, extension] = fileparts(inputFile);
    if ~strcmpi(extension, '.m')
        error('loadDroneInputFile:InvalidFileType', ...
            'Drone input files must be MATLAB .m scripts.');
    end

    droneInputs = executeInputScript(inputFile);
    if ~isstruct(droneInputs) || ~isscalar(droneInputs)
        error('loadDroneInputFile:InvalidDroneInputs', ...
            ['The input script must define droneInputs as one scalar ' ...
             'structure.']);
    end
end

function loadedInputs = executeInputScript(scriptFile)
% A separate function workspace is the isolation boundary. Do not move the
% run call into the public function's caller with evalin or assignin.
    run(scriptFile);
    if ~exist('droneInputs', 'var')
        error('loadDroneInputFile:MissingDroneInputs', ...
            'The input script did not define a variable named droneInputs.');
    end
    loadedInputs = droneInputs;
end
