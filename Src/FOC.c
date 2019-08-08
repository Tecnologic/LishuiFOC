/*
 * FOC.c
 *
 *  Created on: 25.01.2019
 *      Author: Stancecoke
 */
#include "main.h"
#include "config.h"
#include "FOC.h"
#include "stm32f1xx_hal.h"
#include <arm_math.h>

//q31_t	T_halfsample = 0.00003125;
//q31_t	counterfrequency = 64000000;
//q31_t	U_max = (1/_SQRT3)*_U_DC;
long long	temp1;
long long	temp2;
q31_t	temp3;
q31_t	temp4;
q31_t	temp5;
q31_t	temp6;
q31_t z;

q31_t q31_i_q_fil = 0;
q31_t q31_i_d_fil = 0;
q31_t q31_u_d = 0;
q31_t q31_u_q = 0;
volatile long long fl_x1_obs;
volatile long long fl_x2_obs;
q31_t fl_e_alpha_obs;
q31_t fl_e_beta_obs;
q31_t e_log[400][4];

char hall_state_obs=0;
char flux_state_a=0;
char flux_state_b=0;
char flux_state_c=0;





char PI_flag=0;
char Obs_flag=0;



//const q31_t _T = 2048;

TIM_HandleTypeDef htim1;


void FOC_calculation(int16_t int16_i_as, int16_t int16_i_bs, q31_t q31_teta, int16_t int16_i_q_target);
void svpwm(q31_t q31_u_alpha, q31_t q31_u_beta);
q31_t PI_control_i_q (q31_t ist, q31_t soll);
q31_t PI_control_i_d (q31_t ist, q31_t soll);


void observer_update(long long v_a, long long v_b, long long v_c, long long i_a, long long i_b );



void FOC_calculation(int16_t int16_i_as, int16_t int16_i_bs, q31_t q31_teta, int16_t int16_i_q_target)
{

	 q31_t q31_i_alpha = 0;
	 q31_t q31_i_alpha_fil = 0;
	 q31_t q31_i_beta = 0;
	 q31_t q31_i_beta_fil = 0;
	 q31_t q31_u_alpha = 0;
	 q31_t q31_u_beta = 0;
	 q31_t q31_i_d = 0;
	 q31_t q31_i_q = 0;

	 q31_t sinevalue=0, cosinevalue = 0;


	// temp5=(q31_t)int16_i_as;
	// temp6=(q31_t)int16_i_bs;

	// Clark transformation
	arm_clarke_q31((q31_t)int16_i_as, (q31_t)int16_i_bs, &q31_i_alpha, &q31_i_beta);

	arm_sin_cos_q31(q31_teta, &sinevalue, &cosinevalue);
/*
	q31_i_alpha_fil-=q31_i_alpha_fil>>3;
	q31_i_alpha_fil+=q31_i_alpha;
	q31_i_beta_fil-=q31_i_beta_fil>>3;
	q31_i_beta_fil+=q31_i_beta;*/



	// Park transformation
	arm_park_q31(q31_i_alpha, q31_i_beta, &q31_i_d, &q31_i_q, sinevalue, cosinevalue);


	q31_i_q_fil -= q31_i_q_fil>>3;
	q31_i_q_fil += q31_i_q;
	q31_i_d_fil -= q31_i_d_fil>>3;
	q31_i_d_fil += q31_i_d;


	//Control iq

	PI_flag=1;

	/*
	q31_u_q =  PI_control_i_q(q31_i_q_fil>>3, (q31_t) int16_i_q_target);



	//Control id
	q31_u_d = -PI_control_i_d(q31_i_d_fil>>3, 0); //control direct current to zero

	//limit voltage in rotating frame, refer chapter 4.10.1 of UM1052

	q31_t	q31_u_abs = hypot(q31_u_q, q31_u_d); //absolute value of U in static frame
	temp3 = q31_u_abs;


	if (q31_u_abs > _U_MAX){
		q31_u_q = (q31_u_q*_U_MAX)/q31_u_abs; //division!
		q31_u_d = (q31_u_d*_U_MAX)/q31_u_abs; //division!
		temp4=1;
	}
	else temp4=0;
	*/

	//q31_u_q=0;
	//q31_u_d=0;
	//arm_sin_cos_q31(q31_teta, &sinevalue, &cosinevalue);
	//inverse Park transformation
	arm_inv_park_q31(q31_u_d, q31_u_q, &q31_u_alpha, &q31_u_beta, -sinevalue, cosinevalue);
/*
	temp1= q31_i_alpha;
	temp2= q31_i_beta;
	temp3= q31_u_alpha;
    temp4= q31_u_beta;
    */



/*	arm_park_q31((q31_t)fl_e_alpha_obs, (q31_t)fl_e_beta_obs, &q31_e_d_obs, &q31_e_q_obs, sinevalue, cosinevalue);
	q31_e_d_obs_fil -= q31_e_d_obs_fil>>3;
	q31_e_d_obs_fil += q31_e_d_obs;

	q31_delta_teta_obs = PI_control_e_d(q31_e_d_obs_fil>>3, -20000L);

	 */
	//Obs_flag=1;
	//q31_teta_obs=atan2_LUT(fl_e_alpha_obs,fl_e_beta_obs);

	if(!HAL_GPIO_ReadPin(PAS_GPIO_Port, PAS_Pin)&&ui8_debug_state==0)
			{
		e_log[z][0]=temp1;
		e_log[z][1]=temp2;//temp4;
		e_log[z][2]=temp3;//temp5;
		e_log[z][3]=q31_rotorposition_absolute;
		z++;
		if (z>399)
		{z=0;
		ui8_debug_state=2;}
			}
	else {if(ui8_debug_state==2)ui8_debug_state=3;;}
	//call SVPWM calculation
	svpwm(q31_u_alpha, q31_u_beta);
	//temp6=__HAL_TIM_GET_COUNTER(&htim1);
	observer_update((long long)switchtime[0]*(long long)adcData[0]*CAL_V, (long long)switchtime[1]*(long long)adcData[0]*CAL_V, (long long)switchtime[2]*(long long)adcData[0]*CAL_V, (long long)(int16_i_as)*CAL_I, (long long)(int16_i_bs)*CAL_I);


}
//PI Control for quadrature current iq (torque) float operation without division
q31_t PI_control_i_q (q31_t ist, q31_t soll)
{

  q31_t q31_p; //proportional part
  static float flt_q_i = 0; //integral part
  static q31_t q31_q_dc = 0; // sum of proportional and integral part
  q31_p = (soll - ist)*P_FACTOR_I_Q;
  flt_q_i += ((float)(soll - ist))*I_FACTOR_I_Q;

  if ((q31_t)flt_q_i>_U_MAX) flt_q_i=(float)_U_MAX;
  if ((q31_t)flt_q_i<0) flt_q_i = 0 ;

    //avoid too big steps in one loop run
  if (q31_p+(q31_t)flt_q_i>q31_q_dc+5) q31_q_dc+=5;
  else if  (q31_p+(q31_t)flt_q_i<q31_q_dc-5)q31_q_dc-=5;
  else q31_q_dc=q31_p+(q31_t)flt_q_i;


  if (q31_q_dc>_U_MAX) q31_q_dc = _U_MAX;
  if (q31_q_dc<0) q31_q_dc = 0; // allow no negative voltage.

  return (q31_q_dc);
}



//PI Control for direct current id (loss)
q31_t PI_control_i_d (q31_t ist, q31_t soll)
  {
    q31_t q31_p;
    static q31_t q31_d_i = 0;
    static q31_t q31_d_dc = 0;

    q31_p=((soll - ist)*P_FACTOR_I_D)>>4;
    q31_d_i+=((soll - ist)*I_FACTOR_I_D)>>4;

    if (q31_d_i<-127)q31_d_i=-127;
    if (q31_d_i>127)q31_d_i=127;
    //avoid too big steps in one loop run
    if (q31_p+q31_d_i>q31_d_dc+5) q31_d_dc+=5;
    else if  (q31_p+q31_d_i<q31_d_dc-5) q31_d_dc-=5;
    else q31_d_dc=q31_p+q31_d_i;

    if (q31_d_dc>_U_MAX>>2) q31_d_dc = _U_MAX>>2;
    if (q31_d_dc<-(_U_MAX>>2)) q31_d_dc =- (_U_MAX>>2);

    return (q31_d_dc);
  }

void svpwm(q31_t q31_u_alpha, q31_t q31_u_beta)	{

//SVPWM according to chapter 4.9 of UM1052


	q31_t q31_U_alpha = (q31_t)((float)_SQRT3 *(float)_T * (float) q31_u_alpha); //float operation to avoid q31 overflow
	q31_t q31_U_beta = -_T * q31_u_beta;
	q31_t X = q31_U_beta;
	q31_t Y = (q31_U_alpha+q31_U_beta)>>1;
	q31_t Z = (q31_U_beta-q31_U_alpha)>>1;

	//Sector 1 & 4
	if ((Y>=0 && Z<0 && X>0)||(Y < 0 && Z>=0 && X<=0)){
		switchtime[0] = ((_T+X-Z)>>12) + (_T>>1); //right shift 11 for dividing by peroid (=2^11), right shift 1 for dividing by 2
		switchtime[1] = switchtime[0] + (Z>>11);
		switchtime[2] = switchtime[1] - (X>>11);
		//temp4=1;
	}

	//Sector 2 & 5
	if ((Y>=0 && Z>=0) || (Y<0 && Z<0) ){
		switchtime[0] = ((_T+Y-Z)>>12) + (_T>>1);
		switchtime[1] = switchtime[0] + (Z>>11);
		switchtime[2] = switchtime[0] - (Y>>11);
		//temp4=2;
	}

	//Sector 3 & 6
	if ((Y<0 && Z>=0 && X>0)||(Y >= 0 && Z<0 && X<=0)){
		switchtime[0] = ((_T+Y-X)>>12) + (_T>>1);
		switchtime[2] = switchtime[0] - (Y>>11);
		switchtime[1] = switchtime[2] + (X>>11);
		//temp4=3;
	}


}

// See http://cas.ensmp.fr/~praly/Telechargement/Journaux/2010-IEEE_TPEL-Lee-Hong-Nam-Ortega-Praly-Astolfi.pdf
void observer_update(long long v_a, long long v_b, long long v_c, long long i_a, long long i_b ) {

	const long long L = (3LL * INDUCTANCE)>>1;
	long long R = (3LL * RESISTANCE)>>1;
	long long dT = 13LL;
	long long i_c = -i_a -i_b;
	long long vir_temp;
	volatile long long vir_a = 0;
	volatile long long vir_b = 0;
	volatile long long vir_c = 0;
	long long fa_int;
	long long fb_int;
	long long fc_int;

	  // Flux observer. taken from Shane Colton
	  // --------------------------------------------------------------------------
	  // Integral (low-pass filter) of (V - IR).
	  vir_temp = i_a * R;                                  // [+-15] [mV]
	  vir_temp = v_a - vir_temp;                           // [+-18] [mV]
	  vir_temp = vir_temp >> dT;                             // [+-16] [uWb]
	  vir_a -= vir_a>>3;
	  vir_a += vir_temp>>3;

	  /*
	  if(vir_a > VIR_SAT) { vir_a = VIR_SAT; }
	  if(vir_a < -VIR_SAT) { vir_a = -VIR_SAT; }
	  */
	  vir_temp = i_b * R;                                  // [+-15] [mV]
	  vir_temp = v_b - vir_temp;                           // [+-18] [mV]
	  vir_temp = vir_temp >> dT;                             // [+-16] [uWb]
	  vir_b -= vir_b>>3;
	  vir_b += vir_temp>>3;

	  vir_temp = i_c * R;                                  // [+-15] [mV]
	  vir_temp = v_c - vir_temp;                           // [+-18] [mV]
	  vir_temp = vir_temp >> dT;                             // [+-16] [uWb]
	  vir_c -= vir_c>>3;
	  vir_c += vir_temp>>3;

	  // Flux.
	  fa_int = vir_a - i_a * L;
	  /*
	  if(fa_int > F_SAT) { fa_int = F_SAT; }
  	  if(fa_int < -F_SAT) { fa_int = -F_SAT; }
	  */
	  fb_int = vir_b - i_b * L;
	  fc_int = vir_c - i_c * L;
	  temp1=fa_int;
	  temp2=fb_int;
	  temp3=fc_int;

	  if (fa_int>HYST)flux_state_a=1;
	  if (fa_int<-HYST)flux_state_a=0;

	  if (fb_int>HYST)flux_state_b=1;
	  if (fb_int<-HYST)flux_state_b=0;

	  if (fc_int>HYST)flux_state_c=1;
	  if (fc_int<-HYST)flux_state_c=0;

	  hall_state_obs= flux_state_a + (flux_state_b<<1) + (flux_state_c<<2);
	  temp4=hall_state_obs;


}
