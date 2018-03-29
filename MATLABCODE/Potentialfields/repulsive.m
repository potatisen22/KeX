function [Ureptot, Freptot] = repulsive(p0, eta, qp, totalobst, obstpos)
    Ureptot = 0;
    Freptot = 0;
    for i=1:totalobst
        qobst = obstpos(i,:);
        rhoobst = norm(qp-qobst); %Distance to obstacle center
        if rhoobst < p0 %The obstacle is in our sphere of influence %try 1/rho, rho = dist^2
           Urep = 1/2*eta*(1/rhoobst-1/p0)^2;
           Frep =  eta*(1/rhoobst-1/p0)*(1/rhoobst^2)*((qp-qobst)/norm(qp-qobst));%hmm this one?
        else 
            Urep = 0;
            Frep = 0;
        end
        Ureptot = Ureptot+Urep;
        Freptot = Freptot+Frep;
    end
end