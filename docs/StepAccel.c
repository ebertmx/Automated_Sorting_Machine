#include <stdio.h>
int steps = 0;
int vel = 1;
int acc = 0;
int accmax = 5;
int accmin = 0;
int velmax = 10;
int velmin = 0;
int accsteps = 0;
int steps2acc = 10;
int k = 1;

int target = 100;
int ACCFLAG = 0;

int calcacc()
{

    switch (ACCFLAG)
    {
    case (0):
        acc = acc + k;
        if ((steps2acc - accsteps) < (acc / k))
        {
            ACCFLAG = 2;
        }
        if (acc >= accmax)
        {
            ACCFLAG = 1;
        }

        break;
    case (1):
        acc = accmax;
        if ((steps2acc - accsteps) < (accmax / k))
            ACCFLAG = 2;
        break;
    case (2):
        acc = acc - k;
        if (acc <= 0)
        {
            acc = 0;
            ACCFLAG = 0;
            accsteps = 0;
            return 0;
        }
    }

    return 1;
}

int step()
{

    if (steps < steps2acc)
    {
        calcacc();
        vel = vel + acc;
        accsteps = accsteps + 1;
    }

    if ((target - steps2acc) <= steps)
    {
        calcacc();
        vel = vel - acc;
        accsteps = accsteps + 1;
    }

    steps = steps + 1;
    printf("steps = %d    acc = %d     vel = %d  accsteps = %d  ACCFLAG = %d \n", steps, acc, vel, accsteps, ACCFLAG);
    if (steps > target)
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