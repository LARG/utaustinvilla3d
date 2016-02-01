#include <iostream>
#include <sys/time.h>

/****************************************************************************/
/***************************   random stuff    ******************************/
/****************************************************************************/
/* From Andrew Moore's C package                                            */

void set_random_seed_from_clock()
{
    /* initialize the random number seed. */
    timeval tp;
    gettimeofday( &tp, NULL );
    srandom( (unsigned int) tp.tv_usec );
}

int int_random(int n)
{
    static int FirstTime = true;

    if ( FirstTime ) {
        /* initialize the random number seed. */
        timeval tp;
        gettimeofday( &tp, NULL );
        srandom( (unsigned int) tp.tv_usec );
        FirstTime = false;
    }

    if ( n > 2 )
        return( random() % n );
    else if ( n == 2 )
        return( ( (random() % 112) >= 56 ) ? 0 : 1 );
    else if ( n == 1 )
        return(0);
    else
    {
        printf("int_random(%d) ?\n",n);
        std::cerr << ("-E- You called int_random(<=0)") << std::endl;
        return(0);
    }
}

float range_random(float lo, float hi)
{
    int x1 = int_random(10000);
    int x2 = int_random(10000);
    float r = (((float) x1) + 10000.0 * ((float) x2))/(10000.0 * 10000.0);
    return( lo + (hi - lo) * r );
}

int very_random_int(int n)
{
    int result = (int) range_random(0.0,(float)n);  /* rounds down */
    if ( result == n ) result = n-1;
    return(result);
}

