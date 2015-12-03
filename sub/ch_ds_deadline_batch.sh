#!/bin/bash
module load matlab/R2012b
cd ~/proj/matlab-up
matlab -nodesktop -r 'ch_ds_deadline_batch; exit'
