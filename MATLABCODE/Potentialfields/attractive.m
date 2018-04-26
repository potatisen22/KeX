function [Uatt, Fatt] = attractive(d, xi1, xi2, qp, qgoal, rhogoal)
    gradrhogoal = (qp-qgoal)/rhogoal; %gradient of rhogoal
    if rhogoal > d
        Uatt = (1/2*xi1)*rhogoal.^2; %attractive potential if we are far from goal
        Fatt = -xi1*(qp-qgoal)/rhogoal;
    else
        Uatt =  d*xi2*rhogoal; %attractive potential if we are close to goal.
        Fatt = -d*xi2*(qp-qgoal)/rhogoal;
    end
end