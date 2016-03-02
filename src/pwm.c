#include "pwm.h"

int32_t isin_S3(int32_t x)
{   
   // S(x) = x * ( (3<<p) - (x*x>>r) ) >> s
   // n : Q-pos for quarter circle             13
   // A : Q-pos for output                     12
   // p : Q-pos for parentheses intermediate   15
   // r = 2n-p                                 11
   // s = A-1-p-n                              17
   
//   static const int32_t qN = 13, qA= 12, qP= 15, qR= 2*13-15, qS= 13+15+1-12;
   
//   x= x<<(30-qN);          // shift to full s32 range (Q13->Q30)
//   x = x<<(30-13);
   x = x >> 17;

   if( (x^(x<<1)) < 0)     // test for quadrant 1 or 2
      x= (1<<31) - x;
   
//   x= x>>(30-qN);
//   x = x>>(30-13);
   x = x >> 17;
//   return x * ( (3<<qP) - (x*x>>qR) ) >> qS;
   return x * ((3<<15) - (x*x>>11)) >> 17;
}

void setpwm(int32_t sine_deg, int32_t volt_requested, uint32_t volt_dc, uint8_t phases) {

   int32_t pwm_factor = volt_requested / (volt_dc >> 7);
   if(pwm_factor > 128) {
      pwm_factor = 128;
   } else if(pwm_factor < 1) {
      pwm_factor = 1;
   }

   int32_t pwm_a, pwm_b, pwm_c;

   //Calculates sinusoidal PWM value.
   //Shift is not signed, as previous statement returns value between
   //0 and 2000.

   if(phases == 3) {
      pwm_a = (((isin_S3(sine_deg*91) + 4096) * 1000) >> 12);
      pwm_b = (((isin_S3((sine_deg+240)*91) + 4096) * 1000) >> 12);
      pwm_c = (((isin_S3((sine_deg+120)*91) + 4096) * 1000) >> 12);

      pwm_a = (pwm_a * pwm_factor) >> 7;
      pwm_b = (pwm_b * pwm_factor) >> 7;
      pwm_c = (pwm_c * pwm_factor) >> 7;

      if(pwm_a < pwm_b) {
         if(pwm_a < pwm_c) {
            pwm_b -= pwm_a;
            pwm_c -= pwm_a;
            pwm_a = 0;
         } else {
            pwm_a -= pwm_c;
            pwm_b -= pwm_c;
            pwm_c = 0;
         }
      }  else {
         if(pwm_b < pwm_c) {
            pwm_a -= pwm_b;
            pwm_c -= pwm_b;
            pwm_b = 0;
         } else {
            pwm_a -= pwm_c;
            pwm_b -= pwm_c;
            pwm_c = 0;
         }

      }
   } else if (phases == 2) {

      pwm_a = (((isin_S3(sine_deg*91) + 4096) * 1000) >> 12);
      pwm_b = (((isin_S3((sine_deg+180)*91) + 4096) * 1000) >> 12);
   } else {

      pwm_a = (((isin_S3(sine_deg*91) + 4096) * 1000) >> 12);
   }
   if(pwm_a < 30) {
      pwm_a = 30;
   }
   if(pwm_b < 30) {
      pwm_b = 30;
   }
   if(pwm_c < 30) {
      pwm_c = 30;
   }

   timer_set_oc_value(TIM1, TIM_OC1, pwm_a);
   timer_set_oc_value(TIM1, TIM_OC2, pwm_b);
   timer_set_oc_value(TIM1, TIM_OC3, pwm_c);

}

void dcbrake(uint8_t brake_percent) {

   timer_set_oc_value(TIM1, TIM_OC1, 1000+(1000*brake_percent)/100);
   timer_set_oc_value(TIM1, TIM_OC2, 1000);
   timer_set_oc_value(TIM1, TIM_OC3, 1000-(1000*brake_percent)/100);
}

void stop(void) {
   timer_set_oc_value(TIM1, TIM_OC1, 0);
   timer_set_oc_value(TIM1, TIM_OC2, 0);
   timer_set_oc_value(TIM1, TIM_OC3, 0);
}

void disable(void) {
   //Disable gate drivers
   gpio_clear(GPIOB, GPIO1);
   gpio_clear(GPIOB, GPIO2);
   gpio_clear(GPIOB, GPIO3);
}
