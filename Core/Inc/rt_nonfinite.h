/*
 * rt_nonfinite.h
 *
 *  Created on: 26. jun. 2023
 *      Author: pelle
 */

#ifndef RT_NONFINITE_H_
#define RT_NONFINITE_H_

void rt_InitInfAndNaN(size_t realSize);

// Float translations
extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;


#endif /* RT_NONFINITE_H_ */
