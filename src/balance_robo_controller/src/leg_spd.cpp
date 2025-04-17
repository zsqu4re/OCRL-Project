/*
 * File: leg_spd.cpp
 * 转换自原始C代码
 */

/* Include Files */
#include "leg_spd.h"
#include <cmath>

/* Function Definitions */
/*
 * LEG_SPD
 *     SPD = LEG_SPD(DPHI1,DPHI4,PHI1,PHI4)
 *
 * Arguments    : float dphi1
 *                float dphi4
 *                float phi1
 *                float phi4
 *                float spd[2]
 * Return Type  : void
 */
void leg_spd(float dphi1, float dphi4, float phi1, float phi4, float spd[2])
{
  float b_out_tmp;
  float out_tmp;
  float t10_tmp;
  float t12_tmp;
  float t2;
  float t21;
  float t22;
  float t23;
  float t24;
  float t28;
  float t3;
  float t32;
  float t4;
  float t44;
  float t47;
  float t48;
  float t5;
  float t52;
  float t53;
  float t59;
  float t60;
  float t61;
  float t70;
  float t71;
  float t76;
  
  t2 = std::cos(phi1);
  t3 = std::cos(phi4);
  t4 = std::sin(phi1);
  t5 = std::sin(phi4);
  t10_tmp = t2 / 20.0F;
  t12_tmp = t4 / 20.0F;
  t21 = t2 * 0.0105F;
  t22 = t3 * 0.0105F;
  t23 = t4 * 0.0105F;
  t24 = t5 * 0.0105F;
  t28 = t12_tmp - t5 / 20.0F;
  out_tmp = t3 / 20.0F - t10_tmp;
  t32 = t23 - t24;
  b_out_tmp = t22 - t21;
  t44 = t28 * t28 + (out_tmp + 0.06F) * (out_tmp + 0.06F);
  t47 = t2 * t28 / 10.0F + t4 * (out_tmp + 0.06F) / 10.0F;
  t48 = t3 * t28 / 10.0F + t5 * (out_tmp + 0.06F) / 10.0F;
  t52 = 1.0F / ((b_out_tmp + 0.0126F) + t44);
  t53 = t52 * t52;
  t59 = std::sqrt((t32 * t32 + (b_out_tmp + 0.0126F) * (b_out_tmp + 0.0126F)) -
              t44 * t44);
  t60 = 1.0F / t59;
  t61 = (t24 - t23) + t59;
  t28 = std::atan(t52 * t61) * 2.0F;
  t70 = std::cos(t28);
  t71 = std::sin(t28);
  t76 = 1.0F / (t53 * (t61 * t61) + 1.0F);
  t47 = (t23 + t47) * t53 * t61 +
        t52 * (t21 -
               t60 *
                   ((t2 * t32 * 0.021F + t4 * (b_out_tmp + 0.0126F) * 0.021F) -
                    t44 * t47 * 2.0F) /
                   2.0F);
  t28 = (t24 + t48) * t53 * t61 +
        t52 * (t22 -
               t60 *
                   ((t3 * t32 * 0.021F + t5 * (b_out_tmp + 0.0126F) * 0.021F) -
                    t44 * t48 * 2.0F) /
                   2.0F);
  t21 = t12_tmp + t71 * 0.105F;
  out_tmp = t10_tmp + t70 * 0.105F;
  t59 = t70 * t76;
  t23 = t59 * t28;
  t2 = t71 * t76;
  t4 = t2 * t28;
  t28 = -t10_tmp - t70 * 0.105F;
  t60 = (t28 + 0.03F) * (t28 + 0.03F);
  t61 = 1.0F / (t28 + 0.03F);
  t48 = 1.0F / std::sqrt(t21 * t21 + (out_tmp - 0.03F) * (out_tmp - 0.03F));
  t52 = 1.0F / (t21 * t21 + t60);
  t53 = t21 * (1.0F / t60);
  t59 = t10_tmp - t59 * t47 * 0.21F;
  t28 = t12_tmp - t2 * t47 * 0.21F;
  spd[0] =
      dphi4 * t48 * (t21 * t23 * 0.42F - (out_tmp - 0.03F) * t4 * 0.42F) /
          2.0F +
      dphi1 * t48 * (t21 * t59 * 2.0F - (out_tmp - 0.03F) * t28 * 2.0F) / 2.0F;
  spd[1] =
      dphi4 * t60 * t52 * (t61 * (0.0F - t23 * 0.21F) + t53 * (t4 * 0.21F)) -
      dphi1 * t60 * t52 * (t61 * t59 - t53 * t28);
}