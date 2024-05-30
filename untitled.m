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

%[FILENAME, PATHNAME, FILTERINDEX] = uigetfile(FILTERSPEC, TITLE)
[filename, pathname] = uigetfile({'*.jpg;*.png;*.bmp','Image Files (*.jpg, *.png, *.bmp)'});
    if isequal(filename,0)
        return;
    end
    %Citanje slike apsolutnom putanjom
    img = imread(fullfile(pathname, filename));
    handles.img = img;
    %priakzivanaje slike na axes1
    axes(handles.axes1);
    imshow(img);
    guidata(hObject, handles);
% --- Executes on button press in contrastpb.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Ideja je bila da pritiskom na Dugme contrastpb program odradi         %
% binarizaciju sa pragom skaliranim 0-1, medjutim stavio sam u radiobox %
% grupu i ovu modifikaciju, pa dugme contrastpb vise ne postoji         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function contrastpb_Callback(hObject, eventdata, handles)
    if ~isfield(handles, 'img')
        errordlg('Ucitaj sliku prvo');
        return;
    end
    %Prag ili treshold mi odredjuje koji intezitewt pixela ce se videti sa
    %crnom a koji sa belom bojom, threshold je skaliran
    threshold = str2double(get(handles.edit1, 'String'));
    if isnan(threshold) || threshold < 0 || threshold > 1
        errordlg('Unesite skaliran prag (0, 1)');
        return;
    end
    img = handles.img;
    %imbinarize Binarize grayscale 2D image or 3D volume by thresholding.
    %BW = imbinarize(I)
    binarizovana_slika = imbinarize(rgb2gray(img), threshold);

    axes(handles.axes2);
    imshow(binarizovana_slika);
    handles.binarized_img = binarizovana_slika;

% --- Executes on button press in transformacija.
function transformacija_Callback(hObject, eventdata, handles)
%Provera da li je ucitana slika prvo 
    if ~isfield(handles, 'img')
        errordlg('Ucitaj prvo sliku');
        return;
    end
    
    % Uzimanje radiobox transformacije
    modifikacija = get(get(handles.uibuttongroup2, 'SelectedObject'), 'Tag');
    
    % definisanje maski filtera
    % uzeo sam 4 filtera sa vezbi koji su mi bili najbolje radili
    laplace4Mask = [0 1 0; 1 -4 1; 0 1 0];
    laplace8Mask = [1 1 1; 1 -8 1; 1 1 1];
    sobel1Mask = [-1 -2 -1; 0 0 0; 1 2 1];
    sobel2Mask = [-1 0 1; -2 0 2; -1 0 1];
    
    img = handles.img;
    %pretvaranje u sivo
    imgGray = rgb2gray(img);
    
    % transformacija
    %Koristimo funkciju imfilter koji visedimenzionalni niz A filtrira
    %visedim maatricom B i 3. argument je opcioni
    % replicate: To eliminate the zero-padding artifacts around the edge of the image, 
    % imfilter offers an alternative boundary padding method called border replication. In border replication, 
    % the value of any pixel outside the image is determined by replicating the value from the nearest border 
    % pixel. This is illustrated in the following figure.
    % Pikseli izvan granica dobijaju vrednost najbližeg graničnog piksela.
    switch modifikacija
        case 'laplace4'
            transformedImg = imfilter(imgGray, laplace4Mask, 'replicate');
        case 'laplace8'
            transformedImg = imfilter(imgGray, laplace8Mask, 'replicate');
        case 'sobel1'
            transformedImg = imfilter(imgGray, sobel1Mask, 'replicate');
        case 'sobel2'
            transformedImg = imfilter(imgGray, sobel2Mask, 'replicate');
        case 'binarization'
            % uzimam prag / treshold iz edit1
            threshold = str2double(get(handles.edit1, 'String'));
            if isnan(threshold)
                errordlg('Unesi validan prag (0 - 255)');
                return;
            end
            
            % Binarizacija slike
            transformedImg = imbinarize(imgGray, threshold / 255);
    end
    
    % priakz na axes2
    axes(handles.axes2);
    imshow(transformedImg);
    
    % sacuvati sliku az dalju upotrebu u axes2
    handles.transformedImg = transformedImg;
    handles.imageSource = 'transformation';
    guidata(hObject, handles);


function isecitablicu_Callback(hObject, eventdata, handles)
    % provera da li je nesto u axes2 (da li je uradjena modifikacija)
    %u prvoj verziji je bilo 2 mogucnosti editovanja slike: preko
    %laplace/sobela u radiobox grupi ili na button za binarizaciju, ali
    %sada je to samo 1 opcija u transformedImg
    if isfield(handles, 'transformedImg')
        img = handles.transformedImg;
    elseif isfield(handles, 'binarizovana_slika')
        img = handles.binarized_img;
    elseif isfield(handles, 'img')
        img = handles.img;
    else
        errordlg('Nije ucitana slika');
        return;
    end
    
    % u sivo konverzija ako nije radjena vec
    if size(img, 3) == 3
        imgGray = rgb2gray(img);
    else
        imgGray = img;
    end

    % detekcija ivica funkcijom edge. Vise o funkciji u prezentaciji
    %Canny je najgrublja za potiskovanje suma, a u tablicama ne bi trebaloi
    %da ima previse suma pa sam odabrao ovu
    % edge prima sliku u sivoj skali i vracca binarnu sliku gde su pikseli koji pripadaju 
    % ivicama markirani sa 1 (beli pikseli), a ostali pikseli sa 0 (crni pikseli)
    edges = edge(imgGray, 'Canny');
    
    % structuring element
    %rectangle Oblik strukturnog elementa velicina 5x20 pixela
    % pravougaonik se koristi za prosirenje aka dilataciju ivica slike, a
    % edge je nasao ivice vec gore
    se = strel('rectangle', [5, 20]);

    %imdilate ovo se koristi za prosiranje binarne slike elementom
    %pravougaonik sto sam gore definisao
    
    % Dilatacija je u matlabu, po definiciji morfoloska operacija koja prosiri bele regione  na slici
    edgesDilated = imdilate(edges, se);
    
    % bwboundaries pronalazi granice povezanih komponenti u slici koja je
    % binarna, mi smo je vec binarizovali, 'noholes' granice se traze samo 
    % za spoljne regione (bez unutrašnjih rupa).
    % Funkcija bwboundaries vraća dve promenljive:

    %B     niz koji ima koordinate granica svake povezane komponente
    %L    label matrix gde su pikseli svake povezane komponente markirani jedinstvenim celobrojnimsa vrednostima
    [B, L] = bwboundaries(edgesDilated, 'noholes');
    % regionprops vrqaca strukturu nizova gde svaki element sadrzi osobine jedne 
    % povezane komponente ( kao što su bounding box i površina))
    stats = regionprops(L, 'BoundingBox', 'Area');
    
    if isempty(stats)
        errordlg('Nema pronadjenih regiuona linija 202, funkcija je regionprops (proveri jel dobra area!!!)');
        return;
    end
    
    % minimalna povrsina koju ne zelim da detektujem, sto mislim da nije
    % tablica
    minArea = 1000;  % ovo varira po slici, mogao bih napraviti edit2 kao sto unosim
    %threshold za binarizaciju
    stats = stats([stats.Area] > minArea);
    
    % 
    if isempty(stats)
        errordlg('nista nije nadjeno, sve je manje od minarea');
        return;
    end
    
    [~, idx] = max([stats.Area]);
    boundingBox = stats(idx).BoundingBox;
    
    % iseci sliku 
    licensePlate = imcrop(img, boundingBox);
    
    % prikazi je na a axes3
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
    % slika iz axes3
    %children parametar-  sadrzi handleove svih  objekata koji 
    % su deca datog axes objekta.  plotovovi, linije,
    % tekstove, slike nacrtani unutar tog axes objekta. 
    axes3Children = get(handles.axes3, 'Children');
    if isempty(axes3Children)
        errordlg('prvo iseci tablicu na axes3');
        return;
    end
    licensePlate = getimage(axes3Children);
    if isempty(licensePlate)
        errordlg('iseci sliku na axes3');
        return;
    end
    % OCR - PREZENTACIJA
    ocrResults = ocr(licensePlate);
    detectedText = strtrim(ocrResults.Text);
    
    % prikaz rezultata
    set(handles.vrednosttext, 'String', detectedText);



% hObject    handle to tablicautekst (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in rucnoseci.
function rucnoseci_Callback(hObject, eventdata, handles)

    % ovde secem rucno tablicu jer kod nekih slika minarea je losa pa da ne
    % moram da je menjam
    if isfield(handles, 'transformedImg') || isfield(handles, 'binarized_img')
        if isfield(handles, 'transformedImg')
            img = handles.transformedImg;
        else
            img = handles.binarized_img;
        end
    elseif isfield(handles, 'img')
        img = handles.img;
    else
        errordlg('nije ucitana slika');
        return;
    end
    
    
    axes(handles.axes2);
    imshow(img);
    
   
    [x, y] = ginput(2);
    
 
    x1 = min(x);
    y1 = min(y);
    x2 = max(x);
    y2 = max(y);
    %pravim dimenzije crop slike
    width = x2 - x1;
    height = y2 - y1;
    boundingBox = [x1, y1, width, height];
    
    % cropa
    licensePlate = imcrop(img, boundingBox);
    
    % priakz
    axes(handles.axes3);
    imshow(licensePlate);
    handles.licensePlate = licensePlate;
    guidata(hObject, handles);



% hObject    handle to rucnoseci (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
