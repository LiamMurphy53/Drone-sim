function figureHandle = plotDroneConfigurationComparison(comparison)
%PLOTDRONECONFIGURATIONCOMPARISON Plot static multi-layout metrics.
%
% figureHandle = plotDroneConfigurationComparison(comparison)
%
% comparison must be the result of compareDroneConfigurations. Raw panels
% show actual vehicle performance. The normalized panel removes the main
% thrust, size, and inertia scales to make geometry comparisons fairer.

    if ~isstruct(comparison) || ~isscalar(comparison) || ...
            ~isfield(comparison, 'entries') || isempty(comparison.entries)
        error('plotDroneConfigurationComparison:InvalidComparison', ...
            'Provide a nonempty result from compareDroneConfigurations.');
    end
    entries = comparison.entries;
    if ~all(isfield(entries, 'metrics')) || ...
            ~all(isfield(entries, 'label'))
        error('plotDroneConfigurationComparison:MissingFields', ...
            'The comparison entries are missing labels or metrics.');
    end
    metrics = vertcat(entries.metrics);
    labels = {entries.label};

    hoverMargin = vertcat(metrics.minimumHoverMargin_percent);
    worstMoment = vertcat(metrics.worstMoment_Nm);
    bestMoment = vertcat(metrics.bestMoment_Nm);
    worstAcceleration = ...
        vertcat(metrics.worstAngularAcceleration_rad_s2);
    bestAcceleration = ...
        vertcat(metrics.bestAngularAcceleration_rad_s2);
    normalizedScores = [vertcat(metrics.hoverBalanceScore), ...
        vertcat(metrics.normalizedWorstMoment_weightArm), ...
        vertcat(metrics.normalizedWorstAngularAcceleration)];
    anisotropy = [vertcat(metrics.momentAnisotropy), ...
        vertcat(metrics.angularAccelerationAnisotropy)];
    mixerQuality = vertcat(metrics.mixerQualityScore_percent);

    figureHandle = figure('Color', 'w', ...
        'Name', 'Drone configuration comparison', ...
        'Position', [80, 60, 1360, 820]);
    layout = tiledlayout(figureHandle, 2, 3, ...
        'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile(layout);
    bar(hoverMargin, 'FaceColor', [0.22, 0.55, 0.74]);
    grid on;
    ylabel('Two-sided command margin (%)');
    title('Level-hover motor reserve');
    applyConfigurationLabels(labels);

    nexttile(layout);
    bar([worstMoment, bestMoment], 'grouped');
    grid on;
    ylabel('Moment (N m)');
    title('Horizontal moment authority');
    legend({'Worst direction', 'Best direction'}, 'Location', 'best');
    applyConfigurationLabels(labels);

    nexttile(layout);
    bar([worstAcceleration, bestAcceleration], 'grouped');
    grid on;
    ylabel('Angular acceleration (rad/s^2)');
    title('Horizontal angular-acceleration authority');
    legend({'Worst direction', 'Best direction'}, 'Location', 'best');
    applyConfigurationLabels(labels);

    nexttile(layout);
    bar(normalizedScores, 'grouped');
    grid on;
    ylabel('Dimensionless score');
    title('Scale-normalized geometry scores');
    legend({'Hover balance', 'Moment', 'Angular acceleration'}, ...
        'Location', 'best');
    applyConfigurationLabels(labels);

    nexttile(layout);
    bar(anisotropy, 'grouped');
    grid on;
    yline(1, '--', 'Ideal', 'Color', [0.25, 0.25, 0.25]);
    ylabel('Best / worst authority');
    title('Directional unevenness (lower is better)');
    legend({'Moment', 'Angular acceleration'}, 'Location', 'best');
    applyConfigurationLabels(labels);

    nexttile(layout);
    bar(mixerQuality, 'FaceColor', [0.47, 0.36, 0.67]);
    grid on;
    ylim([0, 100]);
    ylabel('Weakest / strongest direction (%)');
    title('Fixed-collective mixer quality (higher is better)');
    applyConfigurationLabels(labels);

    title(layout, ['Frame geometry and complete-vehicle authority  |  ' ...
        'raw and dimensionless views']);
end

function applyConfigurationLabels(labels)
    axesHandle = gca;
    axesHandle.XTick = 1:numel(labels);
    axesHandle.XTickLabel = labels;
    axesHandle.XTickLabelRotation = 20;
end
