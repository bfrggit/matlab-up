#!/bin/bash
module load matlab/R2012b
cd ~/proj/matlab-up
matlab -nodesktop -r 'big_op_sigma_batch; exit'
