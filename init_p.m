% Author: Charles ZHU
% --
% This script prepares basic constants for demo generation and simulations

clear
clc

% P_DIST
%   col 1: Discrete priority values
%   col 2: Corresponding frequencies in demo instance generation
global P_DIST;
P_DIST = [
    1.0 0.1
    0.6 0.3
    0.3 0.6
    ];

% V_MDC
%   Speed of MDC
global V_MDC;
V_MDC = 2;

% INF_PSEUDO
%   Pseudo infinity used for last OP (infinitely far OP)
global INF_PSEUDO;
INF_PSEUDO = 1e+50;
