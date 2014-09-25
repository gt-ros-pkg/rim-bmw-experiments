%% Go through all the files in path
% for getting error statistics

path = '../../data/kf_params/normal1/';
file = strcat(path, '380.csv');
kf_errs(file, 1);