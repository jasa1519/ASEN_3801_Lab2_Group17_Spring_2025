% Lab 2 Function (a)
% input: data file from ASPEN lab
% outputs: t_vec, av_pos_inert, av_att, tar_pos_inert, tar_att
function[t_vec, av_pos_inert, av_att, tar_pos_inert, tar_att] = LoadASPENData(filename)

data = readmatrix(filename);

%% Unpacking Aspen Data

% time vector: 1xn
t_vec = (data(:,1))./100;

% drone position vector: 3xn
pos_av_aspen = [data(:,11) ; data(:,12) ; data(:,13)];

% drone attitude vector: 3xn
att_av_aspen = [data(:, 8) ; data(:, 9) ; data(:, 10)];

% pedestrian position vector: 3xn
pos_tar_aspen = [data(: , 5) ; data(: , 6) ; data(: , 7) ];

% pedestrian attitude vector: 3xn
att_tar_aspen = [data(: , 2) ; data(: , 3) ; data(: , 4)];

%% Converting Data
[av_pos_inert, av_att, tar_pos_inert, tar_att] = ConvertASPENData(pos_av_aspen, att_av_aspen,  pos_tar_aspen, att_tar_aspen);

end
