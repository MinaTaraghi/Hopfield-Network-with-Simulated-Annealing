function varargout = Hopfield(varargin)
% HOPFIELD MATLAB code for Hopfield.fig
%      HOPFIELD, by itself, creates a new HOPFIELD or raises the existing
%      singleton*.
%
%      H = HOPFIELD returns the handle to a new HOPFIELD or the handle to
%      the existing singleton*.
%
%      HOPFIELD('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in HOPFIELD.M with the given input arguments.
%
%      HOPFIELD('Property','Value',...) creates a new HOPFIELD or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Hopfield_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Hopfield_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Hopfield

% Last Modified by GUIDE v2.5 03-Feb-2016 05:24:22

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Hopfield_OpeningFcn, ...
                   'gui_OutputFcn',  @Hopfield_OutputFcn, ...
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


% --- Executes just before Hopfield is made visible.
function Hopfield_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Hopfield (see VARARGIN)
% Choose default command line output for Hopfield
handles.output = hObject;
handles.W=[];
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Hopfield wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Hopfield_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in reset.
function reset_Callback(hObject, eventdata, handles)
% hObject    handle to reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.W = [];
set(handles.nop,'String',num2str(0));
guidata(hObject, handles);


function pixels_Callback(hObject, eventdata, handles)
% hObject    handle to pixels (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pixels as text
%        str2double(get(hObject,'String')) returns contents of pixels as a double
handles.pixels_=str2double(get(hObject,'String'));
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function pixels_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pixels (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in load.
function load_Callback(hObject, eventdata, handles)
% hObject    handle to load (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[fName dirName] = uigetfile('*.bmp;*.tif;*.jpg;*.tiff');
    if fName
        %set(handles.imageSize,'enable','off');
        home=cd(dirName);
        im = imread(fName);
        try
        N = handles.pixels_;
        catch e
            N=7;
        end
        im = fixImage(im,N);
        imagesc(im,'Parent',handles.LPlot);
        colormap('gray');
        cd(home);
    end

% --- Executes on button press in train.
function train_Callback(hObject, eventdata, handles)
% hObject    handle to train (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% Training Network
try
    binbip=handles.binbip_;
catch e
    binbip=1; % Binary
end

try
    N=handles.pixels_;
catch e
    N=50;
end

try
    dataset=handles.dataset_;
catch e
    dataset=1;
end
    
    switch dataset
        case 1
            home=cd('1');
            S(:,1)=double(reshape(imread('X (1).bmp'),[N^2,1]));
            imagesc(imread('X (1).bmp'),'Parent',handles.LPlot);
            colormap('gray');
            pause(0.1);
            S(:,2)=double(reshape(imread('X (2).bmp'),[N^2,1]));
            imagesc(imread('X (2).bmp'),'Parent',handles.LPlot);
            pause(0.1);
            S(:,3)=double(reshape(imread('X (3).bmp'),[N^2,1]));
            imagesc(imread('X (3).bmp'),'Parent',handles.LPlot);
            pause(0.1);
            S(:,4)=double(reshape(imread('X (4).bmp'),[N^2,1]));
            imagesc(imread('X (4).bmp'),'Parent',handles.LPlot);
            pause(0.1);
            S(:,5)=double(reshape(imread('X (5).bmp'),[N^2,1]));
            imagesc(imread('X (5).bmp'),'Parent',handles.LPlot);
            pause(0.1);
            S(:,6)=double(reshape(imread('X (6).bmp'),[N^2,1]));
            imagesc(imread('X (6).bmp'),'Parent',handles.LPlot);
            cd(home);
            if  (binbip == 2)
            S(S==1) = 1;
            S(S==0) = -1;
            end
            Ssi=pinv(S);
            W=S*Ssi;
            set(handles.nop,'String','6');
            
        case 2
            home=cd('2');
            S(:,1)=double(reshape(imread('Y (1).bmp'),[N^2,1]));
            imagesc(imread('Y (1).bmp'),'Parent',handles.LPlot);
            colormap('gray');
            pause(0.1);
            S(:,2)=double(reshape(imread('Y (2).bmp'),[N^2,1]));
            imagesc(imread('Y (2).bmp'),'Parent',handles.LPlot);
            pause(0.1);
            S(:,3)=double(reshape(imread('Y (3).bmp'),[N^2,1]));
            imagesc(imread('Y (3).bmp'),'Parent',handles.LPlot);
            pause(0.1);
            S(:,4)=double(reshape(imread('Y (4).bmp'),[N^2,1]));
            imagesc(imread('Y (4).bmp'),'Parent',handles.LPlot);
            pause(0.1);
            S(:,5)=double(reshape(imread('Y (5).bmp'),[N^2,1]));
            imagesc(imread('Y (5).bmp'),'Parent',handles.LPlot);
            pause(0.1);
            S(:,6)=double(reshape(imread('Y (6).bmp'),[N^2,1]));
            imagesc(imread('Y (6).bmp'),'Parent',handles.LPlot);
            pause(0.1);
            S(:,7)=double(reshape(imread('Y (7).bmp'),[N^2,1]));
            imagesc(imread('Y (7).bmp'),'Parent',handles.LPlot);
            cd(home);
            if  (binbip == 2)
            S(S==1) = 1;
            S(S==0) = -1;
            end
            Ssi=pinv(S);
            W=S*Ssi;
            set(handles.nop,'String','7');            
        case 3
            home=cd('3');
            S(:,1)=double(reshape(imread('X (1).bmp'),[N^2,1]));
            imagesc(imread('X (1).bmp'),'Parent',handles.LPlot);
            colormap('gray');
            pause(0.1);
            S(:,2)=double(reshape(imread('X (2).bmp'),[N^2,1]));
            imagesc(imread('X (2).bmp'),'Parent',handles.LPlot);
            pause(0.1);
            S(:,3)=double(reshape(imread('X (3).bmp'),[N^2,1]));
            imagesc(imread('X (3).bmp'),'Parent',handles.LPlot);
            pause(0.1);
            S(:,4)=double(reshape(imread('X (4).bmp'),[N^2,1]));
            imagesc(imread('X (4).bmp'),'Parent',handles.LPlot);
            pause(0.1);
            S(:,5)=double(reshape(imread('X (5).bmp'),[N^2,1]));
            imagesc(imread('X (5).bmp'),'Parent',handles.LPlot);
            pause(0.1);
            S(:,6)=double(reshape(imread('X (6).bmp'),[N^2,1]));
            imagesc(imread('X (6).bmp'),'Parent',handles.LPlot);
            pause(0.1);
            S(:,7)=double(reshape(imread('X (7).bmp'),[N^2,1]));
            imagesc(imread('X (7).bmp'),'Parent',handles.LPlot);
            pause(0.1);
            S(:,8)=double(reshape(imread('X (8).bmp'),[N^2,1]));
            imagesc(imread('X (8).bmp'),'Parent',handles.LPlot);
            pause(0.1);
            S(:,9)=double(reshape(imread('X (9).bmp'),[N^2,1]));
            imagesc(imread('X (9).bmp'),'Parent',handles.LPlot);
            pause(0.1);
            S(:,10)=double(reshape(imread('X (10).bmp'),[N^2,1]));
            imagesc(imread('X (10).bmp'),'Parent',handles.LPlot);
            cd(home);
            if  (binbip == 2)
            S(S==1) = 1;
            S(S==0) = -1;
            end
            Ssi=pinv(S);
            W=S*Ssi;
            set(handles.nop,'String','10');  
    end
    % Erasing self weight
    for i = 1:N^2
    W(i,i) = 0;
    end
    
    handles.W = W;

guidata(hObject,handles);


% --- Executes on button press in addNoise.
function addNoise_Callback(hObject, eventdata, handles)
% hObject    handle to addNoise (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 im = getimage(handles.LPlot);
 try
 noise_percent=handles.noise_percent_;
 catch e
     noise_percent=10;
 end
N = round( length(im(:))* noise_percent/100 );
    N = max(N,1);   %minimum change one neuron
    ind = ceil(rand(N,1)*length(im(:)));
%    im(ind) = -1*im(ind); %!!!!
    im(ind) = ~im(ind);
    imagesc(im,'Parent',handles.LPlot);
    colormap('gray');
    
% --- Executes on button press in run.
function run_Callback(hObject, eventdata, handles)
% hObject    handle to run (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
try
    t=handles.t0_;
catch e
    t=0.9999;
end

try
    epsilon=handles.epsilon_;
catch e
    epsilon=0.001;
end

try
    alpha=handles.alpha_;
catch e;
    alpha=0.99;
end

try
    d=handles.pixels_;
catch e
    d=50;
end

try
    useSA=handles.sa_;
catch e
    useSA=0;
end

try
    binbip=handles.binbip_;
catch e
    binbip=1;
end

im = getimage(handles.LPlot);
    [rows cols] = size(im);
    N = rows;
    W = handles.W;
    if isempty(W)
        msgbox('Train the Network First!','Error');
        return;        
    end
im = double(reshape(im,[d^2,1]));
if(binbip==2)
    im(im==0)=-1;
end
yold = im;
itr=0;    
while (1)
    itr = itr+1;
    yold= double(reshape(yold,[d^2,1]));
    
    if (useSA)
       
        mat=W*yold;
        deltaE=mat.*(4*yold);
        zetha=rand(length(deltaE),1);
        deltaE(zetha<exp(-deltaE/t))=-1;
        mat(deltaE<=0)=-yold(deltaE<=0);
        mat(deltaE>0)=yold(deltaE>0);
         t=t*alpha;
    else
        mat  = W*yold;
       
        if  (binbip==1)
            mat(mat<=0.5)=0;
            mat(mat==0.5)=yold(mat==0.5);
            mat(mat>0.5)=1;
        elseif  (binbip==2)
            mat(mat<=0)=-1;
            mat(mat==0)=yold(mat==0);
            mat(mat>0)=1;
            
        end
    end
    
    
    ynew=(reshape(mat,[N,N]));
    yold=(reshape(yold,[N,N]));
    mse(itr)=norm(reshape(im,N,N)-ynew,2);
    
    if( (ynew==yold)| mse(itr)<=epsilon )
        imagesc(mat2gray(ynew),'Parent',handles.UPlot); break;
    else
        yold=ynew;
        imagesc(mat2gray(ynew),'Parent',handles.UPlot);
        pause (0.1);
    end
    
    set(handles.minMSE,'String',num2str(min(mse)));
    set(handles.IterNo,'String',num2str(itr));
    set(gcf,'currentAxes',handles.DPlot);
    plot(mse(1:itr));
    title('MSE');
    drawnow;
end



% --- Executes on selection change in binbip.
function binbip_Callback(hObject, eventdata, handles)
% hObject    handle to binbip (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns binbip contents as cell array
%        contents{get(hObject,'Value')} returns selected item from binbip
handles.binbip_=get(hObject,'Value');
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function binbip_CreateFcn(hObject, eventdata, handles)
% hObject    handle to binbip (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function noise_percent_Callback(hObject, eventdata, handles)
% hObject    handle to noise_percent (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of noise_percent as text
%        str2double(get(hObject,'String')) returns contents of noise_percent as a double
handles.noise_percent_=str2double(get(hObject,'String'));
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function noise_percent_CreateFcn(hObject, eventdata, handles)
% hObject    handle to noise_percent (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function t0_Callback(hObject, eventdata, handles)
% hObject    handle to t0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of t0 as text
%        str2double(get(hObject,'String')) returns contents of t0 as a double
handles.t0_=str2double(get(hObject,'String'));
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function t0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to t0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function epsilon_Callback(hObject, eventdata, handles)
% hObject    handle to epsilon (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of epsilon as text
%        str2double(get(hObject,'String')) returns contents of epsilon as a double
handles.epsilon_=str2double(get(hObject,'String'));
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function epsilon_CreateFcn(hObject, eventdata, handles)
% hObject    handle to epsilon (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function alpha_Callback(hObject, eventdata, handles)
% hObject    handle to alpha (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of alpha as text
%        str2double(get(hObject,'String')) returns contents of alpha as a double
handles.alpha_=str2double(get(hObject,'String'));
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function alpha_CreateFcn(hObject, eventdata, handles)
% hObject    handle to alpha (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in save.
function save_Callback(hObject, eventdata, handles)
% hObject    handle to save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in load_net.
function load_net_Callback(hObject, eventdata, handles)
% hObject    handle to load_net (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function save_name_Callback(hObject, eventdata, handles)
% hObject    handle to save_name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of save_name as text
%        str2double(get(hObject,'String')) returns contents of save_name as a double
handles.save_name_=get(hObject,'String');
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function save_name_CreateFcn(hObject, eventdata, handles)
% hObject    handle to save_name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function load_name_Callback(hObject, eventdata, handles)
% hObject    handle to load_name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of load_name as text
%        str2double(get(hObject,'String')) returns contents of load_name as a double
handles.load_name_=get(hObject,'String');
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function load_name_CreateFcn(hObject, eventdata, handles)
% hObject    handle to load_name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function LPlot_CreateFcn(hObject, eventdata, handles)
% hObject    handle to LPlot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate LPlot


function im = fixImage(im,N)
%    if isrgb(im)
% 	if length( size(im) ) == 3
%         im = rgb2gray(im);
%     end
%     im = double(im);
%     m = min(im(:));
%     M = max(im(:));
%     im = (im-m)/(M-m);  %normelizing the image
%     im = imresize(im,[N N],'bilinear');
%     %im = (im > 0.5)*2-1;    %changing image values to -1 & 1
%     im = (im > 0.5);    %changing image values to 0 & 1
im=(im>=1); %Binary


% --- Executes on button press in SA.
function SA_Callback(hObject, eventdata, handles)
% hObject    handle to SA (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of SA
handles.sa_=get(hObject,'Value');
guidata(hObject,handles);


% --- Executes on selection change in dataset.
function dataset_Callback(hObject, eventdata, handles)
% hObject    handle to dataset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns dataset contents as cell array
%        contents{get(hObject,'Value')} returns selected item from dataset
handles.dataset_=get(hObject,'Value');
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function dataset_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dataset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
