#include <stdio.h>
int steps = 0;
int vel = 1;
int acc = 0;
int accmax = 5;
int accmin = 0;
int velmax = 30;
int velmin = 0;
int accsteps = 0;
int steps2acc = 10;
int k = 1;

int target = 100;
int ACCFLAG = 1;

int calcacc()
{

    if ((steps2acc - accsteps) < (acc / k))
    {
        acc = acc - k;
    }
    else if ((steps2acc - accsteps) > (acc / k))
    {
        if (acc < accmax)
        {
            acc = acc + k;
        }
        else
        {
            acc = accmax;
        }
    }

    if (acc <= 0)
    {
        acc = 0;
        accsteps = 0;
        return 0;
    }
    return 1;
}

int step()
{

    if (ACCFLAG && steps <= steps2acc)
    {
        if (calcacc())
        {
            vel = vel + acc;
            if (vel >= velmax)
            {
                vel = velmax;
            }
            accsteps = accsteps + 1;
        }
    }

    if ((target - steps2acc) <= steps)
    {
        if (calcacc())
        {
            vel = vel - acc;
            accsteps = accsteps + 1;
        }
    }
    steps = steps + 1;
    printf("steps = %d    acc = %d     vel = %d  accsteps = %d  ACCFLAG = %d \n", steps, acc, vel, accsteps, ACCFLAG);
    if (steps >= target)
    {
        printf("reached Target");
        return 0;
    }
    return 1;
}

void main()
{
    // accsteps = (1/accmax)* ((velmax- velmin)+ (1/k)*(accmax- accmin));
    while (step())
    {
    }
}