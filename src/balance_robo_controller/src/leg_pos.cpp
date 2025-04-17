/*
 * File: leg_pos.cpp
 * 转换自原始C代码
 */

/* Include Files */
#include "leg_pos.h"
#include <cmath>

/* Function Definitions */
/*
 * LEG_POS
 *     POS = LEG_POS(PHI1,PHI4)
 *
 * Arguments    : float phi1
 *                float phi4
 *                float pos[2]
 * Return Type  : void
 */
void leg_pos(float phi1, float phi4, float pos[2])
{
  float a;
  float b_a;
  float t14;
  float t15;
  float t2;
  float t3;
  float t4;
  float t5;
  float t6;
  float t8_tmp;
  
  t2 = std::cos(phi1);
  t3 = std::cos(phi4);
  t4 = std::sin(phi1);
  t5 = std::sin(phi4);
  t6 = t2 / 20.0F;
  t8_tmp = t4 / 20.0F;
  t14 = t4 * 0.0105F;
  t15 = t5 * 0.0105F;
  t5 = t8_tmp - t5 / 20.0F;
  a = (t3 / 20.0F - t6) + 0.06F;
  b_a = t14 - t15;
  t4 = t3 * 0.0105F - t2 * 0.0105F;
  t5 = t5 * t5 + a * a;
  t4 = std::atan(1.0F / ((t4 + 0.0126F) + t5) *
             ((t15 - t14) +
              std::sqrt((b_a * b_a + (t4 + 0.0126F) * (t4 + 0.0126F)) - t5 * t5))) *
       2.0F;
  t5 = t8_tmp + std::sin(t4) * 0.105F;
  t4 = (t6 + std::cos(t4) * 0.105F) - 0.03F;
  pos[0] = std::sqrt(t5 * t5 + t4 * t4);
  pos[1] = std::atan2(t5, t4);
}