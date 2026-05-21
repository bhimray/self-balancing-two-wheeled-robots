function saveCurrentFigure(fileName)
%SAVECURRENTFIGURE Save the active figure as PNG and MATLAB FIG files.

fig = gcf;
scriptName = getScriptName();
figDir = fullfile(fileparts(mfilename('fullpath')), 'figures', scriptName);

if ~exist(figDir, 'dir')
    mkdir(figDir);
end

safeName = regexprep(fileName, '[^a-zA-Z0-9_-]', '_');
drawnow;

pngPath = fullfile(figDir, [safeName '.png']);

if exist('exportgraphics', 'file') == 2
    exportgraphics(fig, pngPath, 'Resolution', 300);
else
    print(fig, pngPath, '-dpng', '-r300');
end

savefig(fig, fullfile(figDir, [safeName '.fig']));
end

function scriptName = getScriptName()
stack = dbstack;

if numel(stack) >= 3
    [~, scriptName] = fileparts(stack(3).file);
else
    scriptName = 'figures';
end

scriptName = regexprep(scriptName, '[^a-zA-Z0-9_-]', '_');
end
