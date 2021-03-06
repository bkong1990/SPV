%testDistCluters

% clear
% profile on
% 
% nPtsPerClust = 250;
% nClust  = 3;
% totalNumPts = nPtsPerClust*nClust;
% m(:,1) = [1 1]';
% m(:,2) = [-1 -1]';
% m(:,3) = [1 -1]';
% var = .6;
% bandwidth = .75;
% clustMed = [];
% %clustCent;
% 
% 
% x = var*randn(2,nPtsPerClust*nClust);
% %*** build the point set
% for i = 1:nClust
%     x(:,1+(i-1)*nPtsPerClust:(i)*nPtsPerClust)       = x(:,1+(i-1)*nPtsPerClust:(i)*nPtsPerClust) + repmat(m(:,i),1,nPtsPerClust);   
% end

clear;clc
file = textread('../files.txt', '%s', 'delimiter', '\n');
NUM = length(file);
prec = zeros(NUM,1);
rec = zeros(NUM,1);
Total_fn = 0;
Total_tp = 0;
Total_fp = 0;
for index = 1 : length(file)
    fprintf('Processing the %dth image\n',index)
    filename = file{index};
    img = imread(filename);
    %img = img(1:500,1:500);
    [nRows, nCols] = size(img);
    COUNT = sum(sum(img));
    x = zeros(2,COUNT);
    ind = 0;
    for i = 1 : nRows
        for j = 1 : nCols
            count = img(i,j);
            for ptr = 1 : count
                ind = ind + 1;
                x(1,ind) = i;
                x(2,ind) = j;
            end
        end
    end
    
    
    [clustCent,point2cluster,clustMembsCell] = MeanShiftCluster(x,7);
    gt = load(['../in/' filename(1:length(filename)-4) '.txt']);
    
    tol = 8;
    [prec(index), rec(index), tp, fp, fn] = evalDetect(clustCent(2,:)',clustCent(1,:)',...
        gt(:,1), gt(:,2), ones(size(img)),tol);
    
    fprintf('Precision: %f  Recall:%f\n',prec(index), rec(index))
    Total_fn = Total_fn + fn;
    Total_tp = Total_tp + tp;
    Total_fp = Total_fp + fp;
end                                                                                                             

precision = Total_tp / (Total_tp + Total_fp);
recall = Total_tp / (Total_tp + Total_fn);