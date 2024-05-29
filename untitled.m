function varargout = untitled(varargin)

% Edit the above text to modify the response to help untitled

% Last Modified by GUIDE v2.5 29-May-2024 10:30:44

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @untitled_OpeningFcn, ...
                   'gui_OutputFcn',  @untitled_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before untitled is made visible.
function untitled_OpeningFcn(hObject, eventdata, handles, varargin)

handles.output = hObject;

% Update handles structure
guidata(hObject, handles);



% --- Outputs from this function are returned to the command line.
function varargout = untitled_OutputFcn(hObject, eventdata, handles) 

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in loadpb.
function loadpb_Callback(hObject, eventdata, handles)
[filename, pathname] = uigetfile({'*.jpg;*.png;*.bmp','Image Files (*.jpg, *.png, *.bmp)'});
    if isequal(filename,0)
        return;
    end
    img = imread(fullfile(pathname, filename));
    handles.img = img;
    axes(handles.axes1);
    imshow(img);
    guidata(hObject, handles);
% --- Executes on button press in contrastpb.
function contrastpb_Callback(hObject, eventdata, handles)
    if ~isfield(handles, 'img')
        errordlg('Load an image first');
        return;
    end
    threshold = str2double(get(handles.edit1, 'String'));
    if isnan(threshold) || threshold < 0 || threshold > 1
        errordlg('Enter a valid threshold between 0 and 1');
        return;
    end
    img = handles.img;
    binarized_img = imbinarize(rgb2gray(img), threshold);

    axes(handles.axes2);
    imshow(binarized_img);
    handles.binarized_img = binarized_img;

% --- Executes on button press in transformacija.
function transformacija_Callback(hObject, eventdata, handles)
    if ~isfield(handles, 'img')
        errordlg('Load an image first');
        return;
    end
    
    % Get the selected transformation
    selectedTransformation = get(get(handles.uibuttongroup2, 'SelectedObject'), 'Tag');
    
    % Define the masks
    laplace4Mask = [0 1 0; 1 -4 1; 0 1 0];
    laplace8Mask = [1 1 1; 1 -8 1; 1 1 1];
    sobel1Mask = [-1 -2 -1; 0 0 0; 1 2 1];
    sobel2Mask = [-1 0 1; -2 0 2; -1 0 1];
    
    img = handles.img;
    imgGray = rgb2gray(img);
    
    % Apply the selected transformation
    switch selectedTransformation
        case 'laplace4'
            transformedImg = imfilter(imgGray, laplace4Mask, 'replicate');
        case 'laplace8'
            transformedImg = imfilter(imgGray, laplace8Mask, 'replicate');
        case 'sobel1'
            transformedImg = imfilter(imgGray, sobel1Mask, 'replicate');
        case 'sobel2'
            transformedImg = imfilter(imgGray, sobel2Mask, 'replicate');
        case 'binarization'
            % Get threshold value from edit1
            threshold = str2double(get(handles.edit1, 'String'));
            if isnan(threshold)
                errordlg('Please enter a valid numeric threshold');
                return;
            end
            
            % Binarize the image
            transformedImg = imbinarize(imgGray, threshold / 255);
    end
    
    % Display the transformed image in axes2
    axes(handles.axes2);
    imshow(transformedImg);
    
    % Save the transformed image and set the flag
    handles.transformedImg = transformedImg;
    handles.imageSource = 'transformation';
    guidata(hObject, handles);


function isecitablicu_Callback(hObject, eventdata, handles)
    % Check if there is an image in axes2
    if isfield(handles, 'transformedImg')
        img = handles.transformedImg;
    elseif isfield(handles, 'binarized_img')
        img = handles.binarized_img;
    elseif isfield(handles, 'img')
        img = handles.img;
    else
        errordlg('No image loaded');
        return;
    end
    
    % Convert to grayscale if necessary
    if size(img, 3) == 3
        imgGray = rgb2gray(img);
    else
        imgGray = img;
    end

    % Use edge detection to find the edges in the image
    edges = edge(imgGray, 'Canny');
    
    % Dilate the edges to close gaps
    se = strel('rectangle', [5, 20]);
    edgesDilated = imdilate(edges, se);
    
    % Find contours and bounding boxes
    [B, L] = bwboundaries(edgesDilated, 'noholes');
    stats = regionprops(L, 'BoundingBox', 'Area');
    
    if isempty(stats)
        errordlg('No regions found');
        return;
    end
    
    % Filter out small regions that are unlikely to be license plates
    minArea = 1000;  % Adjust this threshold based on your images
    stats = stats([stats.Area] > minArea);
    
    % Assume the largest bounding box is the license plate
    if isempty(stats)
        errordlg('No sufficiently large regions found');
        return;
    end
    
    [~, idx] = max([stats.Area]);
    boundingBox = stats(idx).BoundingBox;
    
    % Crop the image to the bounding box
    licensePlate = imcrop(img, boundingBox);
    
    % Display the license plate in axes3
    axes(handles.axes3);
    imshow(licensePlate);
    handles.licensePlate = licensePlate;
    guidata(hObject, handles);


function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in laplace4.
function laplace4_Callback(hObject, eventdata, handles)
% hObject    handle to laplace4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of laplace4


% --- Executes on button press in laplace8.
function laplace8_Callback(hObject, eventdata, handles)
% hObject    handle to laplace8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of laplace8


% --- Executes on button press in sobel1.
function sobel1_Callback(hObject, eventdata, handles)
% hObject    handle to sobel1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of sobel1


% --- Executes on button press in sobel2.
function sobel2_Callback(hObject, eventdata, handles)
% hObject    handle to sobel2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of sobel2


% --- Executes on button press in tablicautekst.
function tablicautekst_Callback(hObject, eventdata, handles)
    % Retrieve the image from axes3
    axes3Children = get(handles.axes3, 'Children');
    if isempty(axes3Children)
        errordlg('No image found in axes3. Please crop the license plate first.');
        return;
    end
    
    % Assuming the image is the only child of axes3
    licensePlate = getimage(axes3Children);
    
    if isempty(licensePlate)
        errordlg('No image found in axes3. Please crop the license plate first.');
        return;
    end
    
    % Perform OCR on the license plate image
    ocrResults = ocr(licensePlate);
    detectedText = strtrim(ocrResults.Text);
    
    % Display the detected text in the static text field
    set(handles.vrednosttext, 'String', detectedText);



% hObject    handle to tablicautekst (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in rucnoseci.
function rucnoseci_Callback(hObject, eventdata, handles)

    % Check if there is an image in axes2
    if isfield(handles, 'transformedImg') || isfield(handles, 'binarized_img')
        if isfield(handles, 'transformedImg')
            img = handles.transformedImg;
        else
            img = handles.binarized_img;
        end
    elseif isfield(handles, 'img')
        img = handles.img;
    else
        errordlg('No image loaded');
        return;
    end
    
    % Display the image in axes2 to ensure ginput works on the correct image
    axes(handles.axes2);
    imshow(img);
    
    % Get two points from the user
    [x, y] = ginput(2);
    
    % Calculate the bounding box
    x1 = min(x);
    y1 = min(y);
    x2 = max(x);
    y2 = max(y);
    width = x2 - x1;
    height = y2 - y1;
    boundingBox = [x1, y1, width, height];
    
    % Crop the image to the bounding box
    licensePlate = imcrop(img, boundingBox);
    
    % Display the license plate in axes3
    axes(handles.axes3);
    imshow(licensePlate);
    handles.licensePlate = licensePlate;
    guidata(hObject, handles);



% hObject    handle to rucnoseci (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
