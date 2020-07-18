function varargout = beginmeet(varargin)
% BEGINMEET MATLAB code for beginmeet.fig
%      BEGINMEET, by itself, creates a new BEGINMEET or raises the existing
%      singleton*.
%
%      H = BEGINMEET returns the handle to a new BEGINMEET or the handle to
%      the existing singleton*.
%
%      BEGINMEET('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in BEGINMEET.M with the given input arguments.
%
%      BEGINMEET('Property','Value',...) creates a new BEGINMEET or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before beginmeet_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to beginmeet_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help beginmeet

% Last Modified by GUIDE v2.5 01-Jun-2020 14:35:14

% Begin initialization code - DO NOT EDIT

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @beginmeet_OpeningFcn, ...
                   'gui_OutputFcn',  @beginmeet_OutputFcn, ...
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


% --- Executes just before beginmeet is made visible.
function beginmeet_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to beginmeet (see VARARGIN)




 
 



 %colormap gray

 %set(ha,'handlevisibility','off','visible','off');


%% ��������
Fs = 8192;
speed = 0.7;
%% ���ɺ��ϣ��󰶵Ŀ���
y1 = gen_wav(8,0.25 * speed);
y2 = gen_wav(8,0.25 * speed);
y3 = gen_wav(7,0.25 * speed);
y4 = gen_wav(8,0.75 * speed);
y5 = gen_wav(7,0.25 * speed);
y6 = gen_wav(8,0.25 * speed);
y7 = gen_wav(7,0.25 * speed);
y8 = gen_wav(8,1 * speed);
y9 = gen_wav(9,0.75 * speed);
y = [y1,y2,y3,y4,y5,y6,y7,y8,y9];


%% ����һ����Ʒ�������
y1 = gen_wav(7,0.25 * speed);
y2 = gen_wav(7,0.25 * speed);
y3 = gen_wav(6,0.25 * speed);
y4 = gen_wav(7,0.75 * speed);
y5 = gen_wav(6,0.25 * speed);
y6 = gen_wav(7,0.25 * speed);
y7 = gen_wav(6,0.25 * speed);
y8 = gen_wav(7,1 * speed);
y9 = gen_wav(8,1.5 * speed);
y = [y,y1,y2,y3,y4,y5,y6,y7,y8,y9];
%% ���´�ӡ���죬��~
y1 = gen_wav(6,0.5 * speed);
y2 = gen_wav(8,0.5 * speed);
y3 = gen_wav(10,0.5 * speed);
y4 = gen_wav(9,0.5 * speed);
y5 = gen_wav(8,0.5 * speed);
y6 = gen_wav(10,3 * speed);
y7 = gen_wav(11,0.5 * speed);
y8 = gen_wav(10,0.5 * speed);
y9 = gen_wav(9,0.25 * speed);
y10 = gen_wav(8,0.5 * speed);
y = [y,y1,y2,y3,y4,y5,y6,y7,y8,y9,y10];
%% ����õ�壬����д��˭
y1 = gen_wav(8,0.25 * speed);
y2 = gen_wav(8,0.25 * speed);
y3 = gen_wav(7,0.25 * speed);
y4 = gen_wav(8,0.75 * speed);
y5 = gen_wav(7,0.25 * speed);
y6 = gen_wav(8,0.25 * speed);
y7 = gen_wav(7,0.25 * speed);
y8 = gen_wav(8,1 * speed);
y9 = gen_wav(9,0.75 * speed);
y = [y,y1,y2,y3,y4,y5,y6,y7,y8,y9];

%% ������򣬷紵���Խ�
y1 = gen_wav(7,0.25 * speed);
y2 = gen_wav(7,0.25 * speed);
y3 = gen_wav(6,0.25 * speed);
y4 = gen_wav(7,0.75 * speed);
y5 = gen_wav(6,0.25 * speed);
y6 = gen_wav(7,0.25 * speed);
y7 = gen_wav(6,0.25 * speed);
y8 = gen_wav(7,1 * speed);
y9 = gen_wav(8,1.5 * speed);
y = [y,y1,y2,y3,y4,y5,y6,y7,y8,y9];
%% ΢Ц�����Ϸ�
y1 = gen_wav(6,0.5 * speed);
y2 = gen_wav(8,0.5 * speed);
y3 = gen_wav(10,0.5 * speed);
y4 = gen_wav(9,0.5 * speed);
y5 = gen_wav(8,0.5 * speed);
y6 = gen_wav(8,3.5 * speed);
y = [y,y1,y2,y3,y4,y5,y6];
%%��˵���е���׷
y1 = gen_wav(6,0.5 * speed);
y2 = gen_wav(7,0.5 * speed);
y3 = gen_wav(8,0.5 * speed);
y4 = gen_wav(8,0.5 * speed);
y5 = gen_wav(8,0.5 * speed);
y6 = gen_wav(8,0.5 * speed);
y7 = gen_wav(8,1 * speed);
y = [y,y1,y2,y3,y4,y5,y6,y7];
%%������֪�Ѷ���
y1 = gen_wav(6,0.5 * speed);
y2 = gen_wav(7,0.5 * speed);
y3 = gen_wav(8,0.5 * speed);
y4 = gen_wav(8,0.5 * speed);
y5 = gen_wav(8,0.5 * speed);
y6 = gen_wav(9,0.5 * speed);
y7 = gen_wav(9,1 * speed);
y = [y,y1,y2,y3,y4,y5,y6,y7];
%%���ﲻ�������
y1 = gen_wav(5,0.5 * speed);
y2 = gen_wav(6,0.5 * speed);
y3 = gen_wav(7,0.5 * speed);
y4 = gen_wav(7,0.5 * speed);
y5 = gen_wav(7,0.5 * speed);
y6 = gen_wav(7,0.5 * speed);
y7 = gen_wav(7,1 * speed);
y = [y,y1,y2,y3,y4,y5,y6,y7];
%%ֻҪ��鿵���Ҷ
y1 = gen_wav(5,0.5 * speed);
y2 = gen_wav(6,0.5 * speed);
y3 = gen_wav(7,0.5 * speed);
y4 = gen_wav(7,0.5 * speed);
y5 = gen_wav(7,0.5 * speed);
y6 = gen_wav(8,0.5 * speed);
y7 = gen_wav(8,1 * speed);
y = [y,y1,y2,y3,y4,y5,y6,y7];

%% Ӫ��������Լ��
y1 = gen_wav(8,0.5 * speed);
y2 = gen_wav(9,0.5 * speed);
y3 = gen_wav(10,0.5 * speed);
y4 = gen_wav(10,0.5 * speed);
y5 = gen_wav(10,0.5 * speed);
y6 = gen_wav(6,0.5 * speed);
y7 = gen_wav(8,1 * speed);
y = [y,y1,y2,y3,y4,y5,y6,y7];
%%�����¸���һ��
y1 = gen_wav(8,0.5 * speed);
y2 = gen_wav(9,0.5 * speed);
y3 = gen_wav(10,0.5 * speed);
y4 = gen_wav(10,0.5 * speed);
y5 = gen_wav(10,0.5 * speed);
y6 = gen_wav(6,0.5 * speed);
y7 = gen_wav(8,1 * speed);
y = [y,y1,y2,y3,y4,y5,y6,y7];
%%ӵ�����ӵ��ȫ����
y1 = gen_wav(8,0.5 * speed);
y2 = gen_wav(9,0.5 * speed);
y3 = gen_wav(10,0.5 * speed);
y4 = gen_wav(10,0.5 * speed);
y5 = gen_wav(10,0.5 * speed);
y6 = gen_wav(10,1 * speed);
y7 = gen_wav(8,0.5 * speed);
y8 = gen_wav(9,0.5 * speed);
y9 = gen_wav(9,3 * speed);
y = [y,y1,y2,y3,y4,y5,y6,y7,y8,y9];

%% �װ��İ���������������۵ĺ�����
y1 = gen_wav(5,0.5 * speed);
y2 = gen_wav(11,0.5 * speed);
y3 = gen_wav(10,1.5 * speed);
y4 = gen_wav(11,0.5 * speed);
y5 = gen_wav(10,0.5 * speed);
y6 = gen_wav(9,1 * speed);
y7 = gen_wav(8,1.5 * speed);
y8 = gen_wav(9,0.5 * speed);
y9 = gen_wav(10,1 * speed);
y10 = gen_wav(8,1.5 * speed);
y11 = gen_wav(6,1 * speed);
y12 = gen_wav(8,0.5 * speed);
y13 = gen_wav(12,1 * speed);
y14 = gen_wav(8,0.5 * speed);
y15 = gen_wav(10,0.5 * speed);
y16 = gen_wav(10,3 * speed);
y = [y,y1,y2,y3,y4,y5,y6,y7,y8,y9,y10,y11,y12,y13,y14,y15,y16];
%%�װ��ı���������۾�����˵��Ը��
y1 = gen_wav(5,0.5 * speed);
y2 = gen_wav(11,0.5 * speed);
y3 = gen_wav(10,1.5 * speed);
y4 = gen_wav(11,0.5 * speed);
y5 = gen_wav(10,1 * speed);
y6 = gen_wav(9,1 * speed);
y7 = gen_wav(8,1.5 * speed);
y8 = gen_wav(9,0.5 * speed);
y9 = gen_wav(10,1 * speed);
y10 = gen_wav(13,1.5 * speed);
y11 = gen_wav(10,1 * speed);
y12 = gen_wav(6,0.5 * speed);
y13 = gen_wav(8,1 * speed);
y14 = gen_wav(9,1 * speed);
y15 = gen_wav(9,1 * speed);
y16 = gen_wav(8,3 * speed);
y = [y,y1,y2,y3,y4,y5,y6,y7,y8,y9,y10,y11,y12,y13,y14,y15,y16];

    sound(y,Fs)
 



   %% ���ñ���ͼƬ
axes('units','normalized','pos',[0 0 1 1]);
%% ���ð�ťͼƬ
A=imread('123.png');   %��ȡͼƬ
B = imresize(A,0.19);
set(handles.pushbutton1,'CData',B);  %����ť�ı���ͼƬ���ó�A��������ť
 %uistack(ha,'down');
 %ii=imread('begin.jpg');
 %imshow(ii);
 %% ���ͼƬΪ2015
S=char('begin.jpg','2014.jpg','2015.jpg','2016.jpg','2017.jpg','2018.jpg','2019.jpg','forever.jpg');
K=char('123.png','2014-2.jpg','2015-2.jpg','2016-2.jpg','2017-2.jpg','2018-2.jpg','2019-2.jpg','123.png');
Imagename=cellstr(S);
datename=cellstr(K);
axes(handles.axes4);
axes('units','normalized','pos',[0 0 1 1]);
for i = 1:1:8
    I1=imread(Imagename{i});I2=imread(Imagename{i+1});%�л�����
    J1=imread(datename{i+1});%ͼ��ͼ��
    I1=im2double(I1);I2=im2double(I2);
    [x,y,z]=size(I1);
    Im=I1;
    hIm = imshow(I1);
    step=x/100;%���䲽��
    Length=(y-x)/2;
    % ����Ч��һ
    %%���2015�걳��

    %��ͣ5��
    pause(4)
            %%���İ�ťͼ��
    B = imresize(J1,0.19);
    set(handles.pushbutton1,'CData',B); 
    for j=step:step:x
        % �ı�ͼ������
%         Im(1:j,1:Length,:)=I2(x-j+1:x,1:Length,:);
%         Im(x-j+1:x,y-Length+1:y,:)=I2(1:j,y-Length+1:y,:);
%         Im(1:x/2,Length+1:Length+j,:)=I2(1:x/2,y-Length-j+1:y-Length,:);
%         Im(x/2+1:x,y-Length-j+1:y-Length,:)=I2(x/2+1:x,Length+1:Length+j,:);
%         set(hIm,'CData',Im);%����image����CData����ΪIm
%         drawnow ;%�ػ浱ǰͼ�δ���
        Im=I1;
        Im(x-j+1:x,1:Length,:)=I2(1:j,1:Length,:);
        Im(1:j,y-Length+1:y,:)=I2(x-j+1:x,y-Length+1:y,:);
        
        Im(x/2-j/2+1:x/2+j/2,y/2-j/2+1:y/2+j/2,:)=I2(x/2-j/2+1:x/2+j/2,y/2-j/2+1:y/2+j/2,:);
        set(hIm,'CData',Im);%����image����CData����ΪIm
        drawnow ;%�ػ浱ǰͼ�δ���

    end

end



%% ����õ�廨
axes(handles.axes9);%��axes1�ؼ�����ͼ

[x,t] = meshgrid((0:24)/24,(0:575)/575*17*pi-2*pi);
p     = (pi/2)*exp(-t/8/pi);
u     = 1-(1-mod(3.6*t,2*pi)/pi).^4/2;
y     = 2*(x.*(x-1)).^2.*sin(p);
r     = u.*(x.*sin(p)+y.*cos(p));
z     = u.*(x.*cos(p)-y.*sin(p));
y     = r.*sin(t);
x     = r.*cos(t);
%figure('Color','k')
surface(x,y,z,'EdgeColor','none','FaceColor','r')
view(-22,66),axis equal off
light('pos',[-0.25 -0.25 1], 'style','local', 'color',[1 0.84 0.6])
lighting gouraud

% Choose default command line output for beginmeet
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes beginmeet wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = beginmeet_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%% ��L��
axes(handles.axes1);%��axes1�ؼ�����ͼ
a=linspace(0,3,50);
A=1 ./ a;
hold on;
axis([min(a) max(a) min(A) max(A)])
for i=1:length(A)
    plot(a(i),A(i),'*c--','LineWidth',1);
    pause(0.05);
end
hold off;

%ԭͼ
a=0:0.1:3;
A=1 ./ a;
plot(a,A,'*c-','LineWidth',5)%��
axis([min(a) max(a) min(A) max(A)])
axis square off;%�����᲻��ʾ

%% ��O��
axes(handles.axes2);
x1 = -3:0.01:0;
x2 = 0:0.01:3;
%hn = figure('name','���ĵ���','MenuBar','None');
%set(hn,'color','w');
for t = 1:50
    cla
    eval('yleft = (-x1) .^ (2 / 3) + (0.9 * (3.3 - (-x1) .^ 2) .^ 0.5) .* sin(t * pi * (-x1));');
    eval('yright = x2 .^ (2/3) + (0.9 * (3.3 - x2 .^ 2) .^ 0.5) .* sin(t * pi *x2);');
    %eval��������ֱ�Ӱ��ַ�����������ִ�У�ֱ�������ߺ������ұߺ�����ֵ
    
    plot(x1,yleft,'*r',x2,yright,'*r');
    axis([-1.8 1.8 -1.5 2.5]);
    axis off;%ȥ������
    
    pause(0.1);%����ͣ�г̶���Ч��
    %title('��������');%ˢ��
end
clc;
% 
%% ��V��
axes(handles.axes3);
c=linspace(-4,4,50);
C=abs(-2*c);
axis([min(c) max(c) min(C) max(C)])
hold on;

for i=1:length(C)
plot(c(i),C(i),'sm--','LineWidth',1);
pause(0.05);
end
hold off;
%ԭͼ
c=-4:0.01:4;
C=abs(-2*c);
plot(c,C,'sm--','LineWidth',1)%��ĸV
axis([min(c) max(c) min(C) max(C)])
axis square off;%�����᲻��ʾ

% %% ��E��
% axes(handles.axes4);
% d=linspace(-3,3,50);
% D=-1*abs(sin(d));
% axis([min(d) max(d) min(D) max(D)])
% hold on;
% 
% for i=1:length(D)
% plot(d(i),D(i),'r.','LineWidth',10);
% pause(0.05);
% end
% hold off;
% %ԭͼ
% d=-3:0.01:3;
% D=-1*abs(sin(d));
% plot(D,d,'r','LineWidth',5)%��ĸE
% axis square off;%�����᲻��ʾ
% %% ��Y��
% axes(handles.axes5);
% e=linspace(-10,10,50);
% E=log2(abs(e));
% axis([min(e) max(e) min(E) max(E)])
% hold on;
% 
% for i=1:length(E)
% plot(e(i),E(i),'r.','LineWidth',10);
% pause(0.05);
% end
% hold off;
% %ԭͼ
% e=-10:0.01:10;
% E=log2(abs(e));
% plot(e,E,'r','LineWidth',5)%��ĸY
% axis square off;%�����᲻��ʾ


%% ��O��
axes(handles.axes6);
%hn = figure('name','���ĵ���','MenuBar','None');%����ͼƬ,�޲˵���
%set(hn,'color','w');

for n = 1:100
    cla
    t = num2str(n);
    Lf1 = '(-x)^(2/3)+(0.9*(3.3-(-x)^2)^0.5) * sin(';
    Lf2 = '*pi*(-x))';
    Left = [Lf1,t,Lf2];%���ĺ���
    ezplot(Left);
    hold on
    
    Rf1 = 'x^(2/3)+(0.9*(3.3-x^2)^0.5)*sin(';
    Rf2 = '*pi*x)';
    Right = [Rf1,t,Rf2];
    ezplot(Right);%�Ҳ�ĺ���
    hold on
    axis([-2 2 -1.5 2.5]);%�����᷶Χ
    axis off
    
    drawnow;%ˢ������
    %title('���ĵ���')
end
% 
% %% ��U��
% axes(handles.axes7);
% f=linspace(-4,4,50);
% F=2*f.^2;
% axis([min(f) max(f) min(F) max(F)])
% hold on;
% 
% for i=1:length(F)
% plot(f(i),F(i),'r.','LineWidth',10);
% pause(0.05);
% end
% hold off;
% %ԭͼ
% f=-4:0.01:4;
% F=2*f.^2;
% plot(f,F,'r','LineWidth',5)%��ĸU
% axis square off;%�����᲻��ʾ
% 
%% �����Ρ�
axes(handles.axes8);

%ԭͼ
for i = 1:3
    f=@(x,y,z) x.^2.*z.^3+9*y.^2.*z.^3/80-(x.^2+9*y.^2/4+z.^2-1).^3;%�������溯��
    [x,y,z]=meshgrid(-1.5:0.1:1.5);%��ͼ��Χ
    v=f(x,y,z);
    %��ͼ
    h=patch(isosurface(x,y,z,v,0));
    isonormals(x,y,z,v,h)
    set(h,'FaceColor','r','EdgeColor','none');
    title('Programmed By Dylan Wang')
    alpha(0.6)
    grid off;
    axis([-1.5 1.5 -1.5 1.5 -1.5 1.5])
    lighting Gouraud
    h = camlight('left');
    for i = 1:180
        camorbit(1,0)
        camlight(h,'left');
        drawnow;
    end
end


% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes1


% --- Executes on key press with focus on pushbutton1 and none of its controls.
function pushbutton1_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function axes9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes9


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
