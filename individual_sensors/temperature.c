#include <libpynq.h>
#include <math.h>

int r_to_t(double r_t){
  double t;
/*
  
    if(r_t>=25.33)
      t = -19.15*log(r_t)+61.895;
  else
    t = 107.07*exp(-0.145*r_t);
  return (int)round(t);
  */
    const double A = 0.0007984;
    const double B = 0.0002664;
    const double C = 0.00000013009;

    double lnR = log(r_t*1000);

    t = 1.0 / (A + B * lnR + C * lnR * lnR * lnR);

    return t = t - 273.15; 
  }

int main(void) {
  pynq_init();
  adc_init();
  buttons_init();

  double v_out, v_ref;
  double r_t;
  double temperature;
  double R2;
  
  

  while(!get_button_state(BUTTON0)) {
    v_out = adc_read_channel(ADC0);
    v_ref = 3.3;
    R2 = 0.33;
    r_t = (v_ref - v_out) * (R2 / v_out);

    temperature = r_to_t(r_t); // r_to_t computes the temperature from resistance
                               // It is not available in standard pynq library,
                               // So you have to implement it yourself

    printf("V_out: %f, v_ref: %f, r_t: %f, temperature : %f\n", v_out, v_ref, r_t, temperature);

    sleep_msec(1000);
  }
  adc_destroy();
  buttons_destroy();
  pynq_destroy();
  return 0;
}
