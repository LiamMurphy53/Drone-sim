function figureHandle = plotMixerQuality(quality)
%PLOTMIXERQUALITY Visualize normalized allocation geometry metrics.

    requiredFields = {'normalizedWrenchMap', 'normalizedSingularValues', ...
        'perAxisEffectivenessNormalized', ...
        'perAxisFullCommandSpanNormalized', 'fixedCollective', ...
        'motorNames', 'axisLabels'};
    for fieldIndex = 1:numel(requiredFields)
        if ~isfield(quality, requiredFields{fieldIndex})
            error('plotMixerQuality:InvalidQuality', ...
                'quality is missing the %s field.', requiredFields{fieldIndex});
        end
    end

    figureHandle = figure('Color', 'w', 'Name', 'Mixer geometry quality', ...
        'Position', [100, 80, 1320, 800]);
    layout = tiledlayout(figureHandle, 2, 2, ...
        'TileSpacing', 'compact', 'Padding', 'compact');

    mapAxes = nexttile(layout, 1);
    imagesc(mapAxes, quality.normalizedWrenchMap);
    colorLimit = max(abs(quality.normalizedWrenchMap), [], 'all');
    if colorLimit <= 0
        colorLimit = 1;
    end
    clim(mapAxes, [-colorLimit, colorLimit]);
    colormap(mapAxes, blueWhiteRedMap(256));
    colorbar(mapAxes);
    mapAxes.XTick = 1:numel(quality.motorNames);
    mapAxes.XTickLabel = quality.motorNames;
    mapAxes.YTick = 1:4;
    mapAxes.YTickLabel = quality.axisLabels;
    xlabel(mapAxes, 'Motor');
    ylabel(mapAxes, 'Normalized wrench axis');
    title(mapAxes, 'Normalized motor-to-wrench map');
    addMapValues(mapAxes, quality.normalizedWrenchMap, colorLimit);

    singularAxes = nexttile(layout, 2);
    singularBars = [quality.normalizedSingularValues, ...
        [quality.fixedCollective.normalizedSingularValues; NaN]];
    bar(singularAxes, singularBars, 'grouped');
    singularAxes.XTick = 1:4;
    singularAxes.XTickLabel = {'1 (strongest)', '2', '3', '4 (weakest)'};
    ylabel(singularAxes, 'Dimensionless gain');
    title(singularAxes, 'Normalized singular values');
    legend(singularAxes, {'Full Fz/Mx/My/Mz', ...
        'Mx/My/Mz at fixed Fz'}, 'Location', 'best');
    grid(singularAxes, 'on');

    effectivenessAxes = nexttile(layout, 3);
    bar(effectivenessAxes, [quality.perAxisEffectivenessNormalized, ...
        quality.perAxisFullCommandSpanNormalized], 'grouped');
    effectivenessAxes.XTick = 1:4;
    effectivenessAxes.XTickLabel = quality.axisLabels;
    ylabel(effectivenessAxes, 'Normalized effectiveness');
    title(effectivenessAxes, 'Per-axis command effectiveness');
    legend(effectivenessAxes, {'Unit-norm command gain', ...
        'Full command-box span'}, 'Location', 'best');
    grid(effectivenessAxes, 'on');

    weakestAxes = nexttile(layout, 4);
    bar(weakestAxes, ...
        quality.fixedCollective.weakestMomentDirection, 0.62, ...
        'FaceColor', [0.84, 0.35, 0.13]);
    weakestAxes.XTick = 1:3;
    weakestAxes.XTickLabel = quality.momentAxisLabels;
    ylim(weakestAxes, [-1, 1]);
    yline(weakestAxes, 0, '-', 'Color', [0.3, 0.3, 0.3]);
    ylabel(weakestAxes, 'Direction component');
    title(weakestAxes, sprintf([ ...
        'Weakest fixed-Fz moment direction\n' ...
        'condition %.2f, quality %.1f%%'], ...
        quality.fixedCollective.conditionNumber, ...
        quality.fixedCollective.qualityScore_percent));
    grid(weakestAxes, 'on');

    title(layout, sprintf( ...
        'Mixer quality (reference arm %.1f mm)', ...
        1000 * quality.referenceArm_m));
end

function addMapValues(axesHandle, values, colorLimit)
    for rowIndex = 1:size(values, 1)
        for columnIndex = 1:size(values, 2)
            if abs(values(rowIndex, columnIndex)) > 0.52 * colorLimit
                textColor = [1, 1, 1];
            else
                textColor = [0.1, 0.1, 0.1];
            end
            text(axesHandle, columnIndex, rowIndex, ...
                sprintf('%.3f', values(rowIndex, columnIndex)), ...
                'HorizontalAlignment', 'center', 'Color', textColor, ...
                'FontSize', 8);
        end
    end
end

function colorMap = blueWhiteRedMap(numberOfColors)
    halfCount = floor(numberOfColors / 2);
    lower = [linspace(0.12, 1, halfCount).', ...
        linspace(0.32, 1, halfCount).', ones(halfCount, 1)];
    upperCount = numberOfColors - halfCount;
    upper = [ones(upperCount, 1), ...
        linspace(1, 0.24, upperCount).', ...
        linspace(1, 0.16, upperCount).'];
    colorMap = [lower; upper];
end
