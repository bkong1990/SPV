x = double( imread('C1_S07-8825_ROI_01_Sub_01.tif') );
[y,j] = meanShiftPixCluster(x,20,32);