
 clear all 
 close all
 clc

 x = double( imread('C1_S07-8825_ROI_01_Sub_01.tif') );
 y = meanShiftPixCluster(x,20,32);

 