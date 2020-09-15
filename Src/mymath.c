#include "mymath.h"


float atan_f(float Ua,float Ub) {
			int sta = 0,sp = 0;
			float x2,a;
			float x = Ub/Ua;
					/* check up the sign change */
							if(x<0.F){
								x = -x;
								sta |= 1;
							}
					/* check up the invertation */
							if(x>1.F){
								x = 1.F/x;
								sta |= 2;
							}
					/* process shrinking the domain until x<PI/12 */
							while(x>M_PI12)  {
								sp++; 
								a = x+SQRT3; 
								a = 1.F/a; 
								x *= SQRT3; 
								x -= 1.F; 
								x *= a;
							}
					/* calculation core */
							x2 = x*x; 
							a = x2+1.4087812F; 
							a = 0.55913709F/a; 
							a += 0.60310579F;
							a -= 0.05160454F*x2; 
							a *= x;
					/* process until sp=0 */
							while(sp>0){
								a += M_PI6;
								sp--;
							}
					/* invertation took place */
							if(sta&2) 
								a = M_PI2 - a;
					/* sign change took place */
							if(sta&1) 
								a = -a;
							
							if(Ua < 0){
									a += M_PI;
							}
							else 
								if(a<0){
									a += M_2PI;
								}
					return(a);
}

/*-----јлгоритм вычислени€ быстрого обратного квадратного корн€

¬ходные данные: 
number - число от которого вычисл€ем обратный квадратный корень
¬ыходные данные:
обратны  квадратный корень числу number.
*/

float quickSqrt( float number ) {
  long i;
  float x2, y;
  const float threehalfs = 1.5008908F;

  x2 = number*0.5F;
  y  = number;
  i  = * ( long * ) &y;                       // evil floating point bit level hacking
  i  = 0x5f3759df - ( i >> 1 );               // what the fuck? 
  y  = * ( float * ) &i;
  y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
  //y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed

  return y;
}

