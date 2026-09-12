function init_pyenv()
    % INIT_PYENV Configure MATLAB's Python environment for SIH 2026 Pipeline
    %
    % This is the SINGLE shared configuration point for all py.* calls in the
    % Plan A MATLAB/Simulink pipeline. Call this once at the beginning of any
    % script or MATLAB Function block that uses py.* interop.
    %
    % Key decisions:
    % - OutOfProcess execution mode: torch/opencv/ultralytics are native-
    %   extension-heavy; InProcess (default) shares MATLAB's libraries, which
    %   causes crashes. OutOfProcess avoids this at the cost of slightly higher
    %   per-call overhead (~1-2ms per py.* crossing), which is acceptable here.
    % - PYTHONPATH must include the project root so "import pipeline" works.
    % - The .venv location is hardcoded to /home/rayyan/projects/sih_26/.venv,
    %   which is where Phase 0 created it.

    project_root = '/home/rayyan/projects/sih_26';
    venv_python = fullfile(project_root, '.venv', 'bin', 'python3.10');

    % Set PYTHONPATH so "import pipeline" resolves to the project root
    setenv('PYTHONPATH', project_root);

    % Configure pyenv: use the .venv Python, OutOfProcess mode
    % Only configure once per MATLAB session; if already configured differently,
    % this will raise an error. To reconfigure, call: terminate(pyenv)
    try
        pyenv(Version=venv_python, ExecutionMode="OutOfProcess");
    catch ME
        % If already configured, just verify it's the right version
        if contains(ME.identifier, 'MATLAB:py:AlreadyInitialized')
            current_version = pyenv.Version;
            if ~strcmp(current_version, venv_python)
                error('init_pyenv:VersionMismatch', ...
                      'MATLAB Python is configured to %s, but we need %s.\n', ...
                      'Fix: terminate(pyenv); then re-run this script.', ...
                      current_version, venv_python);
            end
            % else: already correctly configured, continue
        else
            rethrow(ME);
        end
    end

    % Minimal verification: confirm Python version and that pipeline module can be imported
    py_version = string(py.sys.version);
    if ~contains(py_version, '3.10')
        warning('init_pyenv:UnexpectedVersion', ...
                'Python version is %s; expected 3.10.x', py_version);
    end

    try
        py.importlib.import_module('pipeline.pipeline');
    catch ME
        error('init_pyenv:PipelineImportFailed', ...
              'Could not import pipeline.pipeline. Full error:\n%s\n', ME.message);
    end

    % Success
    fprintf('✓ init_pyenv: Python %s configured (OutOfProcess mode)\n', char(py.sys.version_info.major) + "." + char(py.sys.version_info.minor));
end
