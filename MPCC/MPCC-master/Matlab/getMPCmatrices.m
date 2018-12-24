% Copyright (C) 2018, ETH Zurich, D-ITET, Kenneth Kuchera, Alexander Liniger
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
% 
%     http://www.apache.org/licenses/LICENSE-2.0
% 
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [X,U,dU,info] = getMPCmatrices(traj,MPC_vars,ModelParams,borders,Xhor,Uhor,x0,u0)
%Comment: borders = new_b

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% For each stage in the horizon compute the necessary system and cost %%%%%
% matricies and safe them in a array of structs 'stage' %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% cost scaling for numerics
costScale = MPC_vars.costScale;
% init stage struct
stage = struct([]);
%% Generate MPCC problem
% initial state (including previus input for input rate cost)

% state is augmented by the inputs and rate inputs are introduced to
% formulate rate input costs and constraints while retaining a block
% spares formulation
% given x_k+1 = A x_k + B u_k
% do the following state augmentation
% s_k = [x_k,u_k-1], v_k = du_k
% with the following linear system
% s_k+1 = [A B;0 I] s_k + [B;I] v_k

stage(1).x0 = x0;
stage(1).u0 = u0;

for i = 1:MPC_vars.N
    Xk = Xhor(:,i);
    Uk = Uhor(:,i);
    % generate quadratic state(-input) cost
    stage(i).Qk = costScale*generateH(traj,MPC_vars,ModelParams,Xk,i);
    % quadratic rate input cost 
    stage(i).Rk = costScale*2*diag([MPC_vars.rdD; MPC_vars.rdDelta; MPC_vars.rdVtheta]);
    % linear state(-input) cost
    stage(i).fk = costScale*generatef(traj,MPC_vars,ModelParams,Xk,i);
    % linearized dynamics
    [stage(i).Ak,stage(i).Bk,stage(i).gk] = getEqualityConstraints(Xk,Uk,MPC_vars,ModelParams);
    % linearized track constraints
    [stage(i).Ck, stage(i).ug, stage(i).lg] = getInequalityConstraints(borders(max(i-1,1),:),MPC_vars,ModelParams);
    % bounds
    [stage(i).lb, stage(i).ub] = getBounds(MPC_vars,ModelParams);
end
% terminal stage
i = MPC_vars.N+1; 
Xk = Xhor(:,i);
% generate quadratic state(-input) cost
stage(i).Qk = costScale*generateH(traj,MPC_vars,ModelParams,Xk,i);
% quadratic rate input cost 
stage(i).Rk = costScale*2*diag([MPC_vars.rD; MPC_vars.rDelta; MPC_vars.rVtheta]);
% linear state(-input) cost
stage(i).fk = costScale*generatef(traj,MPC_vars,ModelParams,Xk,i);
% linearized track constraints
[stage(i).Ck, stage(i).ug, stage(i).lg] = getInequalityConstraints(borders(i-1,:),MPC_vars,ModelParams);
% bounds
[stage(i).lb, stage(i).ub] = getBounds(MPC_vars,ModelParams);
%% Call solver interface
if MPC_vars.interface == 'Yalmip'
    % yalmip based interface (very slow)
    [X,U,dU,info] = YalmipInterface(stage,MPC_vars,ModelParams);
elseif MPC_vars.interface == 'CVX'
    % CVX based interface (slow)
    [X,U,dU,info] = CVXInterface(stage,MPC_vars,ModelParams);
elseif MPC_vars.interface == 'hpipm'
    % hpipm interface (prefered)
    [X,U,dU,info] = hpipmInterface(stage,MPC_vars,ModelParams);
else
    error('invalid optimization interface')
end

end








