function varargout = agv_schedule_mainwindow(varargin)
% AGV_SCHEDULE_MAINWINDOW MATLAB code for agv_schedule_mainwindow.fig
%      AGV_SCHEDULE_MAINWINDOW, by itself, creates a new AGV_SCHEDULE_MAINWINDOW or raises the existing
%      singleton*.
%
%      H = AGV_SCHEDULE_MAINWINDOW returns the handle to a new AGV_SCHEDULE_MAINWINDOW or the handle to
%      the existing singleton*.
%
%      AGV_SCHEDULE_MAINWINDOW('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in AGV_SCHEDULE_MAINWINDOW.M with the given input arguments.
%
%      AGV_SCHEDULE_MAINWINDOW('Property','Value',...) creates a new AGV_SCHEDULE_MAINWINDOW or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before agv_schedule_mainwindow_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to agv_schedule_mainwindow_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help agv_schedule_mainwindow

% Last Modified by GUIDE v2.5 28-Nov-2018 21:58:37

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @agv_schedule_mainwindow_OpeningFcn, ...
                   'gui_OutputFcn',  @agv_schedule_mainwindow_OutputFcn, ...
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


% --- Executes just before agv_schedule_mainwindow is made visible.
function agv_schedule_mainwindow_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to agv_schedule_mainwindow (see VARARGIN)

% Choose default command line output for agv_schedule_mainwindow
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes agv_schedule_mainwindow wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = agv_schedule_mainwindow_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in togglebutton1.
function togglebutton1_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%define a map
%close all ; clear all ;

%function [] = run()
%clear all; clc; close all;
%参数输入
map = input_costmap_from_file();
Node_num = size(map, 1);
Robot_num = 5;
global acc_coef;
acc_coef = 0.5;
task_success = 0;
%机器人模板
empty_robot.index = 0;
empty_robot.position = 0;
empty_robot.velocity = 0;
empty_robot.velocity_limit = 0;
empty_robot.task = [];
empty_robot.path = [];
empty_robot.battery = 0;
robot = repmat(empty_robot, Robot_num, 1);

%路径表模板
empty_path_table.start_time = 0;
empty_path_table.goal_time = 0;
empty_path_table.robot = 0;
path_table = repmat(empty_path_table, Node_num, Node_num);

%初始化机器人
pos_array = zeros(1, Robot_num);
[ robot, path_table ] = initial_robot_state( robot, path_table, Robot_num, map);

%初始化任务集合
task_array = [38, 21, 16, 15, 1, 34,   36];
%初始化系统时间
global global_time;
global_time = 0;
tic;
%主程序入口
while (1)
   %更新系统时间
    global_time = toc;
    %获取任务（已完成）
    
    %任务分配（已完成）
    current_robot = 0;                                 %每次循环为一个机器人安排任务，current_robot记录本次规划的机器人
%     robot_index = randperm(Robot_num);
    for i=1:Robot_num
        if(isempty(robot(i).task))
            current_robot = i;
            robot(current_robot).task = task_array(1);  %将task_array中第一个任务分配给current_robot
            break;
        end
    end
%     if(current_robot == 0)
%         continue;
%     end
    if(current_robot ~= 0)
        %规划路径(已完成)
        temp_path = Dijkstra(robot(current_robot).position, robot(current_robot).task(1), map);
        robot(current_robot).path = path_with_time(1/acc_coef*global_time, temp_path, map);
        %路径协调(未完成)
        [success, path_table, robot] = scheduler(path_table, robot, current_robot, map);
        if(success)
            task_array = [task_array 25];
            task_array(1) = [];
            task_success = task_success + 1;
        else
            task_array = [task_array 25];
            task_array(1) = [];
            robot(i).path = [robot(i).path(1, 1), ceil(1/acc_coef*global_time)];
        end
    end    
    %更新机器人位置
    for i = 1:Robot_num
        temp_path = robot(i).path;
        [row, col] = find(temp_path(:, 1) == robot(i).position);
        if(row == 0)                                               %未获取到机器人在路径中位置
            
        elseif(row == size(temp_path(:, 1), 1))                    %机器人在路径最后
            
        else                                                       %机器人在路径中间 
            if (acc_coef*temp_path(row, 2) < global_time)
                robot(i).position = temp_path(row+1, 1);           %机器人移动到下一位置
            end
        end
    end
    %显示机器人位置
    for i = 1:Robot_num
        pos_array(i) = robot(i).position;
    end
    viewer(pos_array, map, robot, handles);
    pause(1);
    
    
    for i=1:Robot_num
        if(robot(i).position == robot(i).path(size(robot(i).path, 1)) )
            robot(i).task = [];
            robot(i).path = [];
            robot(i).path = [robot(i).position, ceil(1/acc_coef*global_time)];
        end
        
    end
    if(length(unique(task_array)) == 1)
        break;
    end
     if(isempty(task_array))
         break;
     end
end


    



% --- Executes on button press in communication_setting_button.
function communication_setting_button_Callback(hObject, eventdata, handles)
% hObject    handle to communication_setting_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
t=tcpip('127.0.0.1', 30000,'NetworkRole','server');
% Wait for connection
disp('Waiting for connection');
fopen(t);
disp('Connection OK');
% Read data from the socket
N = 0;
while N < 10
DataReceived=fread(t,2);
set(handles.edit3, 'string', DataReceived);
pause(1);
N = N + 1;
end
fclose(t);

% Hint: get(hObject,'Value') returns toggle state of communication_setting_button


% --- Executes on button press in speed_setting_button.
function speed_setting_button_Callback(hObject, eventdata, handles)
% hObject    handle to speed_setting_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of speed_setting_button


% --- Executes on selection change in listbox2.
function listbox2_Callback(hObject, eventdata, handles)
% hObject    handle to listbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox2


% --- Executes during object creation, after setting all properties.
function listbox2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function control_port_Callback(hObject, eventdata, handles)
% hObject    handle to control_port (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of control_port as text
%        str2double(get(hObject,'String')) returns contents of control_port as a double


% --- Executes during object creation, after setting all properties.
function control_port_CreateFcn(hObject, eventdata, handles)
% hObject    handle to control_port (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function IP_for_Test_Callback(hObject, eventdata, handles)
% hObject    handle to IP_for_Test (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of IP_for_Test as text
%        str2double(get(hObject,'String')) returns contents of IP_for_Test as a double


% --- Executes during object creation, after setting all properties.
function IP_for_Test_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IP_for_Test (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
