function [Uatt, Fatt] = attractive(d, xi, qp, qgoal, rhogoal)
    gradrhogoal = (qp-qgoal)/rhogoal; %gradient of rhogoal
    if rhogoal > d
        Uatt = (1/2*xi)*rhogoal.^2; %attractive potential if we are far from goal
        Fatt = -xi*(qp-qgoal)/rhogoal;
    else
        Uatt =  d*xi*rhogoal; %attractive potential if we are close to goal.
        Fatt = -d*xi*(qp-qgoal)/rhogoal;
    end
end