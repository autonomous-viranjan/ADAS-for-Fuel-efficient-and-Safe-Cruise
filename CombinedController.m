yalmip('clear')
clear, clc
load('fuel_eff.mat')
Am=A/M; Bm=B/M; Cm=C/M;
N = 5;
% Preceding vehicle
%Vp = [10*ones(1,N), 10*ones(1,N), 10:2:28, 28:-2:20, 20:-1:16, 15*ones(1,10)]; 
%Vp = [10:2:20, 20 20 20 20];
Vp = [10*ones(1,N), 11:15, 12:2:20, 14:18, 17:21];
%%
u = sdpvar(ones(1,N+1),ones(1,N+1));
v = sdpvar(ones(1,N+1),ones(1,N+1));
d = sdpvar(ones(1,N+1),ones(1,N+1));

constraints = [];
objective = 0;
for k = 1:N
 objective = objective + sf.p10*v{k} + sf.p01*(u{k}-Am-Bm*v{k}-Cm*v{k}^2) ...
                       + sf.p20*v{k}^2 + sf.p11*v{k}*(u{k}-Am-Bm*v{k}-Cm*v{k}^2) ...
                       + sf.p02*(u{k}-Am-Bm*v{k}-Cm*v{k}^2)^2 + sf.p30*v{k}^3 ...
                       + sf.p21*(v{k}^2)*(u{k}-Am-Bm*v{k}-Cm*v{k}^2) ...
                       + sf.p12*v{k}*((u{k}-Am-Bm*v{k}-Cm*v{k}^2)^2) ...
                       + sf.p03*(u{k}-Am-Bm*v{k}-Cm*v{k}^2)^3 ;
 constraints = [constraints, v{k+1} == (1-Bm)*v{k} - Cm*(v{k}^2) - Am + u{k},
                             d{k+1} == d{k} + (Vp(k) - v{k})];                                
 constraints = [constraints, -2.2 <= u{k}<= 2.2,
                               0<= v{k}<=100,
                         (5+0.1*v{k})<=d{k}<=(15+0.1*v{k}),
                             -1<=u{k+1}-u{k}<=1];
end

%%
controller = optimizer(constraints, objective,[],[v{1};d{1}],[u{:}]);
%% Simulation
v0 = 10;
d0 = 15;
a0 = 2;
vel = 10;
dis = 15;
uimpl = [];
for time=1:4
    uin = controller{[vel;dis]};
    U = tracker(uin,v0,a0,N,Am,Bm,Cm);
    dis = dis + Vp((time-1)*N+1) - vel;
    vel = (1-Bm)*vel - Cm*vel^2 - Am + U(1);    
    uimpl = [uimpl;U(1)];
end
