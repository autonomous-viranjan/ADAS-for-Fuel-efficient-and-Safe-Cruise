function utotal = tracker(uin,v0,a0,N,Am,Bm,Cm)
uref = uin(1:end-1);

vp = [v0 zeros(1,N)];
ap = [a0 zeros(1,N)];
vshow = [v0 zeros(1,N)];
for i=1:N
    vshow(i+1)=(1-(Bm))*vshow(i)-(Cm)*vshow(i)^2-(Am)+uin(i);
end
% Prediction using data-driven sparse regression based model
for j=1:N
    vp(j+1)=0.9383*vp(j)+2.18*ap(j)+0.0615*vshow(j);
    ap(j+1)=-0.03*vp(j)+0.3711*ap(j)+0.0299*vshow(j);
end
up = ap + Cm*vp.^2 + Bm*vp + Am*ones(1,N+1);
ud = up(2:end);
n = length(uref);

% Reference Track
cvx_begin
    variable x(n)
    minimize( norm( x + (ud' - uref'), 2 ) )
    subject to        
        norm( x, inf ) <= 0.3;
cvx_end

utotal = x'+ud;
end