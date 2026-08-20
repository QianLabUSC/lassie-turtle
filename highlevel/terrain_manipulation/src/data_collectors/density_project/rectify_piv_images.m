function rectify_piv_images
%RECTIFY_PIV_IMAGES Manually rectify oblique PIV images using four corners.
%
% Board used here:
%   7 checker columns across 174 mm
%   10 checker rows across 249 mm
%   Output scale: 2 pixels/mm (0.5 mm/pixel)
%
% Select the four extreme INTERNAL checker intersections, one checker inward
% from each outer board edge. Do not select the inset ArUco marker corners.
%
% The target and moving particles must be in the same physical plane.
% Keep the camera, focus, zoom, and image resolution unchanged.

    validExtensions = {'.png', '.jpg', '.jpeg', '.tif', '.tiff', '.bmp'};

    % Measured board properties.
    checkerColumns = 7;
    checkerRows = 10;
    gridWidthMM = 174;
    gridHeightMM = 249;
    pixelsPerMM = 2;
    checkerWidthMM = gridWidthMM / checkerColumns;
    checkerHeightMM = gridHeightMM / checkerRows;

    calibrationMode = questdlg( ...
        'Create a new calibration or apply a saved calibration?', ...
        'Calibration mode', 'Create new calibration', ...
        'Load saved calibration', 'Cancel', 'Load saved calibration');
    sourceCalibrationDataFile = '';

    if strcmp(calibrationMode, 'Create new calibration')
        isNewCalibration = true;

        [calibrationName, calibrationPath] = uigetfile( ...
            {'*.png;*.jpg;*.jpeg;*.tif;*.tiff;*.bmp', 'Image files'}, ...
            'Select the calibration-target image');
        if isequal(calibrationName, 0)
            return
        end

        calibrationFile = fullfile(calibrationPath, calibrationName);
        calibrationImage = imread(calibrationFile);
        outputSize = [size(calibrationImage, 1), size(calibrationImage, 2)];
        outputReference = imref2d(outputSize);

        cornerNames = {'TOP-LEFT INTERNAL', 'TOP-RIGHT INTERNAL', ...
                       'BOTTOM-RIGHT INTERNAL', 'BOTTOM-LEFT INTERNAL'};

        while true
        selectionFigure = figure( ...
            'Name', 'Manual checkerboard-corner selection', ...
            'NumberTitle', 'off', 'WindowState', 'maximized');
        selectionAxes = axes('Parent', selectionFigure, ...
            'Position', [0.04, 0.10, 0.92, 0.85]);
        imshow(calibrationImage, [], 'Parent', selectionAxes);
        hold(selectionAxes, 'on')
        axesToolbar = axtoolbar(selectionAxes, ...
            {'zoom', 'pan', 'restoreview'});
        axesToolbar.Expanded = 'on';

        imagePoints = zeros(4, 2);
        for pointIndex = 1:4
            title(selectionAxes, { ...
                sprintf('Corner %d of 4: %s', ...
                    pointIndex, cornerNames{pointIndex}), ...
                ['Zoom and pan first. Then press the button and click ', ...
                 'the clear intersection where FOUR checker cells meet, ', ...
                 'one checker inward from both nearby outer edges.']});

            selectButton = uicontrol('Parent', selectionFigure, ...
                'Style', 'pushbutton', ...
                'String', ['Select ', cornerNames{pointIndex}], ...
                'Units', 'normalized', ...
                'Position', [0.38, 0.015, 0.24, 0.055], ...
                'FontSize', 12, 'FontWeight', 'bold', ...
                'Callback', @(~, ~) uiresume(selectionFigure));

            uiwait(selectionFigure)
            if ~isgraphics(selectionFigure)
                return
            end
            if isgraphics(selectButton)
                delete(selectButton)
            end

            zoom(selectionFigure, 'off')
            pan(selectionFigure, 'off')
            figure(selectionFigure)
            axes(selectionAxes)
            [x, y] = ginput(1);
            if isempty(x)
                close(selectionFigure)
                return
            end

            imagePoints(pointIndex, :) = [x, y];
            plot(selectionAxes, x, y, 'yo', 'MarkerSize', 10, ...
                'LineWidth', 2, 'MarkerFaceColor', 'red');
            text(selectionAxes, x + 8, y, sprintf('%d', pointIndex), ...
                'Color', 'yellow', 'FontSize', 12, 'FontWeight', 'bold');
        end

        plot(selectionAxes, ...
            [imagePoints(:, 1); imagePoints(1, 1)], ...
            [imagePoints(:, 2); imagePoints(1, 2)], ...
            'y-', 'LineWidth', 1.5);

        % Physical coordinates of the four selected internal intersections.
        % They are one checker inward from every outer edge.
        selectedCornersMM = [ ...
            checkerWidthMM,                 checkerHeightMM; ...
            gridWidthMM - checkerWidthMM,   checkerHeightMM; ...
            gridWidthMM - checkerWidthMM,   gridHeightMM - checkerHeightMM; ...
            checkerWidthMM,                 gridHeightMM - checkerHeightMM];

        % Locate the projected physical center of the board. This keeps the
        % rectified board near its original image location.
        boardToImageTransform = fitgeotform2d( ...
            selectedCornersMM, imagePoints, 'projective');
        gridCenterPixels = transformPointsForward( ...
            boardToImageTransform, [gridWidthMM / 2, gridHeightMM / 2]);

        gridWidthPixels = gridWidthMM * pixelsPerMM;
        gridHeightPixels = gridHeightMM * pixelsPerMM;
        gridLeft = gridCenterPixels(1) - gridWidthPixels / 2;
        gridRight = gridCenterPixels(1) + gridWidthPixels / 2;
        gridTop = gridCenterPixels(2) - gridHeightPixels / 2;
        gridBottom = gridCenterPixels(2) + gridHeightPixels / 2;

        selectedOutputPointsPixels = [ ...
            gridLeft + checkerWidthMM * pixelsPerMM, ...
                gridTop + checkerHeightMM * pixelsPerMM; ...
            gridRight - checkerWidthMM * pixelsPerMM, ...
                gridTop + checkerHeightMM * pixelsPerMM; ...
            gridRight - checkerWidthMM * pixelsPerMM, ...
                gridBottom - checkerHeightMM * pixelsPerMM; ...
            gridLeft + checkerWidthMM * pixelsPerMM, ...
                gridBottom - checkerHeightMM * pixelsPerMM];

        projectiveTransform = fitgeotform2d( ...
            imagePoints, selectedOutputPointsPixels, 'projective');
        rectifiedCalibration = imwarp(calibrationImage, ...
            projectiveTransform, 'OutputView', outputReference, ...
            'FillValues', 0);

        previewFigure = figure('Name', 'Manual rectification preview', ...
            'NumberTitle', 'off', 'WindowState', 'maximized');
        imshow(rectifiedCalibration, []);
        axis on
        axis image
        hold on

        % Overlay the expected 7-by-10 checker boundaries.
        for columnIndex = 0:checkerColumns
            gridX = gridLeft + ...
                columnIndex * checkerWidthMM * pixelsPerMM;
            plot([gridX, gridX], [gridTop, gridBottom], '-', ...
                'Color', [1, 1, 0], 'LineWidth', 0.75);
        end
        for rowIndex = 0:checkerRows
            gridY = gridTop + ...
                rowIndex * checkerHeightMM * pixelsPerMM;
            plot([gridLeft, gridRight], [gridY, gridY], '-', ...
                'Color', [1, 1, 0], 'LineWidth', 0.75);
        end

        xlabel('Image x (pixels)')
        ylabel('Image y (pixels)')
        title(['Manual rectification preview: the yellow grid should ', ...
               'match all checker boundaries']);

        choice = questdlg('Does the rectified checkerboard look correct?', ...
            'Check manual rectification', 'Process images', ...
            'Redo points', 'Cancel', 'Process images');

        if strcmp(choice, 'Process images')
            close(selectionFigure)
            break
        elseif strcmp(choice, 'Redo points')
            close(selectionFigure)
            close(previewFigure)
        else
            close(selectionFigure)
            close(previewFigure)
            return
        end
        end

    elseif strcmp(calibrationMode, 'Load saved calibration')
        isNewCalibration = false;
        [calibrationDataName, calibrationDataPath] = uigetfile( ...
            {'*.mat', 'MATLAB calibration files'}, ...
            'Select rectification_calibration.mat');
        if isequal(calibrationDataName, 0)
            return
        end

        sourceCalibrationDataFile = fullfile( ...
            calibrationDataPath, calibrationDataName);
        loadedCalibration = load(sourceCalibrationDataFile);
        requiredFields = {'projectiveTransform', 'outputReference', ...
            'imagePoints', 'selectedOutputPointsPixels', ...
            'selectedCornersMM', 'checkerColumns', 'checkerRows', ...
            'gridWidthMM', 'gridHeightMM', 'checkerWidthMM', ...
            'checkerHeightMM', 'pixelsPerMM'};
        for fieldIndex = 1:numel(requiredFields)
            if ~isfield(loadedCalibration, requiredFields{fieldIndex})
                error('Saved calibration is missing the field "%s".', ...
                    requiredFields{fieldIndex});
            end
        end

        projectiveTransform = loadedCalibration.projectiveTransform;
        outputReference = loadedCalibration.outputReference;
        outputSize = outputReference.ImageSize;
        imagePoints = loadedCalibration.imagePoints;
        selectedOutputPointsPixels = ...
            loadedCalibration.selectedOutputPointsPixels;
        selectedCornersMM = loadedCalibration.selectedCornersMM;
        checkerColumns = loadedCalibration.checkerColumns;
        checkerRows = loadedCalibration.checkerRows;
        gridWidthMM = loadedCalibration.gridWidthMM;
        gridHeightMM = loadedCalibration.gridHeightMM;
        checkerWidthMM = loadedCalibration.checkerWidthMM;
        checkerHeightMM = loadedCalibration.checkerHeightMM;
        pixelsPerMM = loadedCalibration.pixelsPerMM;
        if isfield(loadedCalibration, 'calibrationFile')
            calibrationFile = loadedCalibration.calibrationFile;
        else
            calibrationFile = sourceCalibrationDataFile;
        end

        calibrationPath = calibrationDataPath;
        fprintf('Loaded calibration: %s\n', sourceCalibrationDataFile);
        fprintf('Required frame size: %d rows by %d columns.\n', ...
            outputSize(1), outputSize(2));
        fprintf('Spatial scale: %.6f mm/pixel.\n', 1 / pixelsPerMM);

    else
        return
    end

    inputDirectory = uigetdir(calibrationPath, ...
        'Select the folder containing the experimental PIV frames');
    if isequal(inputDirectory, 0)
        return
    end

    % Use a new folder so manual results cannot mix with prior attempts.
    outputDirectory = fullfile(inputDirectory, 'rectified_manual');
    if ~isfolder(outputDirectory)
        mkdir(outputDirectory)
    end
    calibrationInfoDirectory = fullfile( ...
        outputDirectory, 'calibration_info');
    if ~isfolder(calibrationInfoDirectory)
        mkdir(calibrationInfoDirectory)
    end

    directoryListing = dir(inputDirectory);
    numberProcessed = 0;
    for fileIndex = 1:numel(directoryListing)
        fileInfo = directoryListing(fileIndex);
        if fileInfo.isdir
            continue
        end

        [~, ~, extension] = fileparts(fileInfo.name);
        if ~any(strcmpi(extension, validExtensions))
            continue
        end

        inputFile = fullfile(inputDirectory, fileInfo.name);
        if strcmp(inputFile, calibrationFile)
            continue
        end

        frame = imread(inputFile);
        if size(frame, 1) ~= outputSize(1) || ...
                size(frame, 2) ~= outputSize(2)
            warning('Skipping %s because its dimensions differ from calibration.', ...
                fileInfo.name);
            continue
        end

        rectifiedFrame = imwarp(frame, projectiveTransform, ...
            'OutputView', outputReference, 'FillValues', 0);

        % Preserve the exact original filename and extension.
        outputFile = fullfile(outputDirectory, fileInfo.name);
        if any(strcmpi(extension, {'.tif', '.tiff'}))
            imwrite(rectifiedFrame, outputFile, 'Compression', 'none');
        elseif any(strcmpi(extension, {'.jpg', '.jpeg'}))
            imwrite(rectifiedFrame, outputFile, 'Quality', 100);
        else
            imwrite(rectifiedFrame, outputFile);
        end

        numberProcessed = numberProcessed + 1;
        fprintf('Rectified %d: %s\n', numberProcessed, fileInfo.name);
    end

    rectifiedCalibrationFile = fullfile( ...
        calibrationInfoDirectory, 'rectification_preview.png');
    if isNewCalibration
        imwrite(rectifiedCalibration, rectifiedCalibrationFile);
    else
        sourcePreviewFile = fullfile( ...
            fileparts(sourceCalibrationDataFile), ...
            'rectification_preview.png');
        if isfile(sourcePreviewFile)
            copyfile(sourcePreviewFile, rectifiedCalibrationFile, 'f');
        end
    end

    millimetresPerPixelX = 1 / pixelsPerMM;
    millimetresPerPixelY = 1 / pixelsPerMM;
    calibrationDataFile = fullfile( ...
        calibrationInfoDirectory, 'rectification_calibration.mat');
    save(calibrationDataFile, 'projectiveTransform', 'outputReference', ...
        'imagePoints', 'selectedOutputPointsPixels', 'selectedCornersMM', ...
        'checkerColumns', 'checkerRows', 'gridWidthMM', 'gridHeightMM', ...
        'checkerWidthMM', 'checkerHeightMM', 'pixelsPerMM', ...
        'millimetresPerPixelX', 'millimetresPerPixelY', ...
        'calibrationFile', 'sourceCalibrationDataFile');

    fprintf('\nFinished: %d experimental images rectified.\n', ...
        numberProcessed);
    fprintf('Output folder: %s\n', outputDirectory);
    fprintf(['Spatial scale: %.6f mm/pixel horizontally, ', ...
        '%.6f mm/pixel vertically.\n'], ...
        millimetresPerPixelX, millimetresPerPixelY);

    msgbox(sprintf([ ...
        'Finished rectifying %d images.\n\n', ...
        'Output folder:\n%s\n\n', ...
        'Scale: %.6f mm/pixel (x), %.6f mm/pixel (y)'], ...
        numberProcessed, outputDirectory, ...
        millimetresPerPixelX, millimetresPerPixelY), ...
        'Manual rectification complete');
end