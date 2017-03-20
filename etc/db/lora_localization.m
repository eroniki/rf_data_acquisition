% This script calls necessary scripts to run a localization test on Hancock
% dataset. The localization is based on LDPL model and least squares
% estimation.
%% Clear up and close all existing figures
clear all; close all; clc;
%% Preprocess the data
preprocess
%% Run the test
verbose = 1;
path_loss
[h, p] = kstest(z_score(error_map(:)))
[h, p] = kstest(z_score(error_map_ind(:)))
%% Visualize and save the resulting figures
save_figures = 1;
% visualize
