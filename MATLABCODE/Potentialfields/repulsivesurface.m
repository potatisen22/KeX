function [Ureptot, Freptot] = repulsivesurface (p0, eta, qp, totalobst, obstpos, obstrad)
    Ureptot = 0;
    Freptot = 0;
    for i=1:totalobst
        qobst = obstpos(i,:);
        rad = obstrad(i,1);
        rhoobst = norm(qp-qobst); %Distance to obstacle center
        rhoobstsurf = rhoobst - rad; %Distance to obstacle surface
        vectosurf = rhoobstsurf * (qp-qobst)/rhoobst;
        
        if rhoobstsurf < p0 %The obstacle is in our sphere of influence %try 1/rho, rho = dist^2
           Urep = 1/2*eta*(1/rhoobstsurf-1/p0)^2;
           Frep =  eta*(1/rhoobstsurf-1/p0)*(1/rhoobstsurf^2)*(vectosurf/norm(vectosurf));
        else 
            Urep = 0;
            Frep = 0;
        end
        Ureptot = Ureptot+Urep;
        Freptot = Freptot+Frep;
    end
end