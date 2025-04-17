/*
 * File: leg_conv.cpp
 * 转换自原始C代码
 */

/* Include Files */
#include "leg_conv.h"
#include <cmath>

/* Function Definitions */
/*
 * LEG_CONV
 *     T = LEG_CONV(F,TP,PHI1,PHI4)
 *
 * Arguments    : float F
 *                float Tp
 *                float phi1
 *                float phi4
 *                float T[2]
 * Return Type  : void
 */
void leg_conv(float F, float Tp, float phi1, float phi4, float T[2])
{
  float a;
  float b_out;
  float b_out_tmp;
  float out;
  float out_tmp;
  float t10_tmp;
  float t11;
  float t12_tmp;
  float t13_tmp;
  float t14_tmp_tmp;
  float t16;
  float t17_tmp_tmp;
  float t19;
  float t21_tmp;
  float t25;
  float t26;
  float t27_tmp;
  float t28;
  float t2_tmp;
  float t30;
  float t3_tmp;
  float t42;
  float t4_tmp;
  float t50;
  float t52;
  float t53;
  float t56_tmp;
  float t57;
  float t58;
  float t59_tmp;
  float t60;
  float t63;
  float t65;
  float t69;
  float t6_tmp;
  float t72_tmp;
  float t73;
  float t77;
  float t7_tmp;
  float t8;
  float t80_tmp;
  float t81_tmp;
  float t89;
  float t9;
  float t99;
  
  t3_tmp = std::cos(phi1);
  t4_tmp = std::cos(phi4);
  t12_tmp = t3_tmp * 0.0105F;
  t13_tmp = t4_tmp * 0.0105F;
  t2_tmp = -t12_tmp + t13_tmp;
  t14_tmp_tmp = t3_tmp * 0.05F;
  out_tmp = -t14_tmp_tmp + t4_tmp * 0.05F;
  t6_tmp = std::sin(phi1);
  t7_tmp = std::sin(phi4);
  t17_tmp_tmp = t6_tmp * 0.05F;
  t8 = t17_tmp_tmp - t7_tmp * 0.05F;
  t16 = (out_tmp + 0.06F) * (out_tmp + 0.06F);
  t19 = t8 * t8;
  t9 = t16 + t19;
  t10_tmp = t7_tmp * 0.0105F;
  t21_tmp = t6_tmp * 0.0105F;
  t11 = -t10_tmp + t21_tmp;
  t25 = ((t2_tmp + 0.0126F) * (t2_tmp + 0.0126F) - t9 * t9) + t11 * t11;
  t26 = std::sqrt(t25);
  t27_tmp = ((t2_tmp + t16) + t19) + 0.0126F;
  t28 = 1.0F / t27_tmp;
  a = t17_tmp_tmp +
      std::sin(std::atan(t28 * ((t6_tmp * -0.0105F + t10_tmp) + t26)) * 2.0F) * 0.105F;
  t30 = (t10_tmp - t21_tmp) + t26;
  t16 = std::atan(t28 * t30) * 2.0F;
  t80_tmp = std::cos(t16);
  out = t14_tmp_tmp + t80_tmp * 0.105F;
  t42 = t17_tmp_tmp - t7_tmp * 0.05F;
  b_out_tmp = -t14_tmp_tmp + t4_tmp * 0.05F;
  t19 = t42 * t42;
  t50 = (b_out_tmp + 0.06F) * (b_out_tmp + 0.06F);
  t52 = t6_tmp * (b_out_tmp + 0.06F) * 0.1F;
  t53 = t3_tmp * t42 * 0.1F;
  t56_tmp = -t12_tmp + t13_tmp;
  t89 = ((t56_tmp + t19) + t50) + 0.0126F;
  t57 = (-t21_tmp + t26) + t10_tmp;
  t58 = 1.0F / (t89 * t89);
  t59_tmp = std::sin(t16);
  t60 = 1.0F / t26;
  t63 = t21_tmp - t10_tmp;
  t65 = t19 + t50;
  t69 = 1.0F / t89;
  t72_tmp = t57 * t58;
  t73 = (t12_tmp -
         t60 *
             ((t6_tmp * (t56_tmp + 0.0126F) * 0.021F + t3_tmp * t63 * 0.021F) -
              t65 * (t52 + t53) * 2.0F) *
             0.5F) *
            t69 +
        t72_tmp * ((t21_tmp + t52) + t53);
  t77 = 1.0F / (t58 * (t57 * t57) + 1.0F);
  t81_tmp = t17_tmp_tmp + t59_tmp * 0.105F;
  b_out = -t14_tmp_tmp - t80_tmp * 0.105F;
  t19 = (out_tmp + 0.06F) * t6_tmp * 0.1F;
  t89 = t3_tmp * t8 * 0.1F;
  t16 = 1.0F / (t27_tmp * t27_tmp);
  t26 = 1.0F / (t16 * (t30 * t30) + 1.0F);
  t99 = 1.0F / std::sqrt(t25);
  t50 = t30 * t16;
  t30 = t28 * (t12_tmp -
               (((t2_tmp + 0.0126F) * t6_tmp * 0.021F + t3_tmp * t11 * 0.021F) -
                t9 * (t19 + t89) * 2.0F) *
                   t99 * 0.5F) +
        t50 * ((t21_tmp + t19) + t89);
  t25 = t80_tmp * t26;
  t27_tmp = t59_tmp * t26;
  t57 = (b_out + 0.03F) * (b_out + 0.03F);
  t58 = (out - 0.03F) * (out - 0.03F);
  t19 = t7_tmp * (b_out_tmp + 0.06F) * 0.1F;
  t16 = t42 * t4_tmp * 0.1F;
  t52 = t69 * (t13_tmp - t60 *
                             ((t7_tmp * (t56_tmp + 0.0126F) * 0.021F +
                               t4_tmp * t63 * 0.021F) -
                              t65 * (t19 + t16) * 2.0F) *
                             0.5F) +
        t72_tmp * ((t10_tmp + t19) + t16);
  t53 = t17_tmp_tmp + t59_tmp * 0.105F;
  t19 = (out_tmp + 0.06F) * t7_tmp * 0.1F;
  t16 = t4_tmp * t8 * 0.1F;
  t89 = t28 * (t13_tmp - t99 *
                             (((t2_tmp + 0.0126F) * t7_tmp * 0.021F +
                               t4_tmp * t11 * 0.021F) -
                              t9 * (t19 + t16) * 2.0F) *
                             0.5F) +
        t50 * ((t10_tmp + t19) + t16);
  t26 = 1.0F / (b_out + 0.03F);
  t50 = t81_tmp * (1.0F / t57);
  t16 = Tp * t57 * (1.0F / (t57 + t81_tmp * t81_tmp));
  t19 = (t14_tmp_tmp + t80_tmp * 0.105F) - 0.03F;
  T[0] = F *
             (t19 * (t17_tmp_tmp - t59_tmp * t73 * t77 * 0.21F) * 2.0F -
              t53 * (t14_tmp_tmp - t73 * t77 * t80_tmp * 0.21F) * 2.0F) *
             -0.5F / std::sqrt(t58 + a * a) -
         t16 * (t26 * (t14_tmp_tmp - t25 * t30 * 0.21F) -
                t50 * (t17_tmp_tmp - t27_tmp * t30 * 0.21F));
  T[1] =
      F *
          (t59_tmp * t77 * t19 * t52 * 0.42F -
           t77 * t80_tmp * t52 * t53 * 0.42F) *
          -0.5F / std::sqrt(t58 + t81_tmp * t81_tmp) +
      t16 * (t26 * (0.0F - t25 * t89 * 0.21F) + t50 * (t27_tmp * t89 * 0.21F));
}