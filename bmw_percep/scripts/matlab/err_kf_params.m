%% Go through all the files in path
% for getting error statistics

path = '../../data/kf_params/normal1/';
ext = '.csv';

file_nos = [0, 398];
kf_jerks = [.005, .005];
errs = [];
for i=file_nos(1):file_nos(2)
    file = strcat(path, num2str(i), ext);
    errs = [errs; kf_errs(file,0)];
end