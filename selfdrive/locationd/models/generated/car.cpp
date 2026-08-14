#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6280992013641613331) {
   out_6280992013641613331[0] = delta_x[0] + nom_x[0];
   out_6280992013641613331[1] = delta_x[1] + nom_x[1];
   out_6280992013641613331[2] = delta_x[2] + nom_x[2];
   out_6280992013641613331[3] = delta_x[3] + nom_x[3];
   out_6280992013641613331[4] = delta_x[4] + nom_x[4];
   out_6280992013641613331[5] = delta_x[5] + nom_x[5];
   out_6280992013641613331[6] = delta_x[6] + nom_x[6];
   out_6280992013641613331[7] = delta_x[7] + nom_x[7];
   out_6280992013641613331[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7070051882147892355) {
   out_7070051882147892355[0] = -nom_x[0] + true_x[0];
   out_7070051882147892355[1] = -nom_x[1] + true_x[1];
   out_7070051882147892355[2] = -nom_x[2] + true_x[2];
   out_7070051882147892355[3] = -nom_x[3] + true_x[3];
   out_7070051882147892355[4] = -nom_x[4] + true_x[4];
   out_7070051882147892355[5] = -nom_x[5] + true_x[5];
   out_7070051882147892355[6] = -nom_x[6] + true_x[6];
   out_7070051882147892355[7] = -nom_x[7] + true_x[7];
   out_7070051882147892355[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_7380527807372010906) {
   out_7380527807372010906[0] = 1.0;
   out_7380527807372010906[1] = 0.0;
   out_7380527807372010906[2] = 0.0;
   out_7380527807372010906[3] = 0.0;
   out_7380527807372010906[4] = 0.0;
   out_7380527807372010906[5] = 0.0;
   out_7380527807372010906[6] = 0.0;
   out_7380527807372010906[7] = 0.0;
   out_7380527807372010906[8] = 0.0;
   out_7380527807372010906[9] = 0.0;
   out_7380527807372010906[10] = 1.0;
   out_7380527807372010906[11] = 0.0;
   out_7380527807372010906[12] = 0.0;
   out_7380527807372010906[13] = 0.0;
   out_7380527807372010906[14] = 0.0;
   out_7380527807372010906[15] = 0.0;
   out_7380527807372010906[16] = 0.0;
   out_7380527807372010906[17] = 0.0;
   out_7380527807372010906[18] = 0.0;
   out_7380527807372010906[19] = 0.0;
   out_7380527807372010906[20] = 1.0;
   out_7380527807372010906[21] = 0.0;
   out_7380527807372010906[22] = 0.0;
   out_7380527807372010906[23] = 0.0;
   out_7380527807372010906[24] = 0.0;
   out_7380527807372010906[25] = 0.0;
   out_7380527807372010906[26] = 0.0;
   out_7380527807372010906[27] = 0.0;
   out_7380527807372010906[28] = 0.0;
   out_7380527807372010906[29] = 0.0;
   out_7380527807372010906[30] = 1.0;
   out_7380527807372010906[31] = 0.0;
   out_7380527807372010906[32] = 0.0;
   out_7380527807372010906[33] = 0.0;
   out_7380527807372010906[34] = 0.0;
   out_7380527807372010906[35] = 0.0;
   out_7380527807372010906[36] = 0.0;
   out_7380527807372010906[37] = 0.0;
   out_7380527807372010906[38] = 0.0;
   out_7380527807372010906[39] = 0.0;
   out_7380527807372010906[40] = 1.0;
   out_7380527807372010906[41] = 0.0;
   out_7380527807372010906[42] = 0.0;
   out_7380527807372010906[43] = 0.0;
   out_7380527807372010906[44] = 0.0;
   out_7380527807372010906[45] = 0.0;
   out_7380527807372010906[46] = 0.0;
   out_7380527807372010906[47] = 0.0;
   out_7380527807372010906[48] = 0.0;
   out_7380527807372010906[49] = 0.0;
   out_7380527807372010906[50] = 1.0;
   out_7380527807372010906[51] = 0.0;
   out_7380527807372010906[52] = 0.0;
   out_7380527807372010906[53] = 0.0;
   out_7380527807372010906[54] = 0.0;
   out_7380527807372010906[55] = 0.0;
   out_7380527807372010906[56] = 0.0;
   out_7380527807372010906[57] = 0.0;
   out_7380527807372010906[58] = 0.0;
   out_7380527807372010906[59] = 0.0;
   out_7380527807372010906[60] = 1.0;
   out_7380527807372010906[61] = 0.0;
   out_7380527807372010906[62] = 0.0;
   out_7380527807372010906[63] = 0.0;
   out_7380527807372010906[64] = 0.0;
   out_7380527807372010906[65] = 0.0;
   out_7380527807372010906[66] = 0.0;
   out_7380527807372010906[67] = 0.0;
   out_7380527807372010906[68] = 0.0;
   out_7380527807372010906[69] = 0.0;
   out_7380527807372010906[70] = 1.0;
   out_7380527807372010906[71] = 0.0;
   out_7380527807372010906[72] = 0.0;
   out_7380527807372010906[73] = 0.0;
   out_7380527807372010906[74] = 0.0;
   out_7380527807372010906[75] = 0.0;
   out_7380527807372010906[76] = 0.0;
   out_7380527807372010906[77] = 0.0;
   out_7380527807372010906[78] = 0.0;
   out_7380527807372010906[79] = 0.0;
   out_7380527807372010906[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_6083295923350312382) {
   out_6083295923350312382[0] = state[0];
   out_6083295923350312382[1] = state[1];
   out_6083295923350312382[2] = state[2];
   out_6083295923350312382[3] = state[3];
   out_6083295923350312382[4] = state[4];
   out_6083295923350312382[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6083295923350312382[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6083295923350312382[7] = state[7];
   out_6083295923350312382[8] = state[8];
}
void F_fun(double *state, double dt, double *out_7234405902953304843) {
   out_7234405902953304843[0] = 1;
   out_7234405902953304843[1] = 0;
   out_7234405902953304843[2] = 0;
   out_7234405902953304843[3] = 0;
   out_7234405902953304843[4] = 0;
   out_7234405902953304843[5] = 0;
   out_7234405902953304843[6] = 0;
   out_7234405902953304843[7] = 0;
   out_7234405902953304843[8] = 0;
   out_7234405902953304843[9] = 0;
   out_7234405902953304843[10] = 1;
   out_7234405902953304843[11] = 0;
   out_7234405902953304843[12] = 0;
   out_7234405902953304843[13] = 0;
   out_7234405902953304843[14] = 0;
   out_7234405902953304843[15] = 0;
   out_7234405902953304843[16] = 0;
   out_7234405902953304843[17] = 0;
   out_7234405902953304843[18] = 0;
   out_7234405902953304843[19] = 0;
   out_7234405902953304843[20] = 1;
   out_7234405902953304843[21] = 0;
   out_7234405902953304843[22] = 0;
   out_7234405902953304843[23] = 0;
   out_7234405902953304843[24] = 0;
   out_7234405902953304843[25] = 0;
   out_7234405902953304843[26] = 0;
   out_7234405902953304843[27] = 0;
   out_7234405902953304843[28] = 0;
   out_7234405902953304843[29] = 0;
   out_7234405902953304843[30] = 1;
   out_7234405902953304843[31] = 0;
   out_7234405902953304843[32] = 0;
   out_7234405902953304843[33] = 0;
   out_7234405902953304843[34] = 0;
   out_7234405902953304843[35] = 0;
   out_7234405902953304843[36] = 0;
   out_7234405902953304843[37] = 0;
   out_7234405902953304843[38] = 0;
   out_7234405902953304843[39] = 0;
   out_7234405902953304843[40] = 1;
   out_7234405902953304843[41] = 0;
   out_7234405902953304843[42] = 0;
   out_7234405902953304843[43] = 0;
   out_7234405902953304843[44] = 0;
   out_7234405902953304843[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_7234405902953304843[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_7234405902953304843[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7234405902953304843[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7234405902953304843[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_7234405902953304843[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_7234405902953304843[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_7234405902953304843[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_7234405902953304843[53] = -9.8100000000000005*dt;
   out_7234405902953304843[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_7234405902953304843[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_7234405902953304843[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7234405902953304843[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7234405902953304843[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_7234405902953304843[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_7234405902953304843[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_7234405902953304843[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7234405902953304843[62] = 0;
   out_7234405902953304843[63] = 0;
   out_7234405902953304843[64] = 0;
   out_7234405902953304843[65] = 0;
   out_7234405902953304843[66] = 0;
   out_7234405902953304843[67] = 0;
   out_7234405902953304843[68] = 0;
   out_7234405902953304843[69] = 0;
   out_7234405902953304843[70] = 1;
   out_7234405902953304843[71] = 0;
   out_7234405902953304843[72] = 0;
   out_7234405902953304843[73] = 0;
   out_7234405902953304843[74] = 0;
   out_7234405902953304843[75] = 0;
   out_7234405902953304843[76] = 0;
   out_7234405902953304843[77] = 0;
   out_7234405902953304843[78] = 0;
   out_7234405902953304843[79] = 0;
   out_7234405902953304843[80] = 1;
}
void h_25(double *state, double *unused, double *out_1871314369284315990) {
   out_1871314369284315990[0] = state[6];
}
void H_25(double *state, double *unused, double *out_6271385423409671601) {
   out_6271385423409671601[0] = 0;
   out_6271385423409671601[1] = 0;
   out_6271385423409671601[2] = 0;
   out_6271385423409671601[3] = 0;
   out_6271385423409671601[4] = 0;
   out_6271385423409671601[5] = 0;
   out_6271385423409671601[6] = 1;
   out_6271385423409671601[7] = 0;
   out_6271385423409671601[8] = 0;
}
void h_24(double *state, double *unused, double *out_4554521396056145628) {
   out_4554521396056145628[0] = state[4];
   out_4554521396056145628[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4098735824404172035) {
   out_4098735824404172035[0] = 0;
   out_4098735824404172035[1] = 0;
   out_4098735824404172035[2] = 0;
   out_4098735824404172035[3] = 0;
   out_4098735824404172035[4] = 1;
   out_4098735824404172035[5] = 0;
   out_4098735824404172035[6] = 0;
   out_4098735824404172035[7] = 0;
   out_4098735824404172035[8] = 0;
   out_4098735824404172035[9] = 0;
   out_4098735824404172035[10] = 0;
   out_4098735824404172035[11] = 0;
   out_4098735824404172035[12] = 0;
   out_4098735824404172035[13] = 0;
   out_4098735824404172035[14] = 1;
   out_4098735824404172035[15] = 0;
   out_4098735824404172035[16] = 0;
   out_4098735824404172035[17] = 0;
}
void h_30(double *state, double *unused, double *out_1714127700933186845) {
   out_1714127700933186845[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8789718381916920228) {
   out_8789718381916920228[0] = 0;
   out_8789718381916920228[1] = 0;
   out_8789718381916920228[2] = 0;
   out_8789718381916920228[3] = 0;
   out_8789718381916920228[4] = 1;
   out_8789718381916920228[5] = 0;
   out_8789718381916920228[6] = 0;
   out_8789718381916920228[7] = 0;
   out_8789718381916920228[8] = 0;
}
void h_26(double *state, double *unused, double *out_8113169218347035991) {
   out_8113169218347035991[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2529882104535615377) {
   out_2529882104535615377[0] = 0;
   out_2529882104535615377[1] = 0;
   out_2529882104535615377[2] = 0;
   out_2529882104535615377[3] = 0;
   out_2529882104535615377[4] = 0;
   out_2529882104535615377[5] = 0;
   out_2529882104535615377[6] = 0;
   out_2529882104535615377[7] = 1;
   out_2529882104535615377[8] = 0;
}
void h_27(double *state, double *unused, double *out_7110654802722524775) {
   out_7110654802722524775[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6614955070116495317) {
   out_6614955070116495317[0] = 0;
   out_6614955070116495317[1] = 0;
   out_6614955070116495317[2] = 0;
   out_6614955070116495317[3] = 1;
   out_6614955070116495317[4] = 0;
   out_6614955070116495317[5] = 0;
   out_6614955070116495317[6] = 0;
   out_6614955070116495317[7] = 0;
   out_6614955070116495317[8] = 0;
}
void h_29(double *state, double *unused, double *out_770617848031524410) {
   out_770617848031524410[0] = state[1];
}
void H_29(double *state, double *unused, double *out_9146794347478239204) {
   out_9146794347478239204[0] = 0;
   out_9146794347478239204[1] = 1;
   out_9146794347478239204[2] = 0;
   out_9146794347478239204[3] = 0;
   out_9146794347478239204[4] = 0;
   out_9146794347478239204[5] = 0;
   out_9146794347478239204[6] = 0;
   out_9146794347478239204[7] = 0;
   out_9146794347478239204[8] = 0;
}
void h_28(double *state, double *unused, double *out_2108981535061679130) {
   out_2108981535061679130[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4217550709161781838) {
   out_4217550709161781838[0] = 1;
   out_4217550709161781838[1] = 0;
   out_4217550709161781838[2] = 0;
   out_4217550709161781838[3] = 0;
   out_4217550709161781838[4] = 0;
   out_4217550709161781838[5] = 0;
   out_4217550709161781838[6] = 0;
   out_4217550709161781838[7] = 0;
   out_4217550709161781838[8] = 0;
}
void h_31(double *state, double *unused, double *out_7785047395955064363) {
   out_7785047395955064363[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1903674002302263901) {
   out_1903674002302263901[0] = 0;
   out_1903674002302263901[1] = 0;
   out_1903674002302263901[2] = 0;
   out_1903674002302263901[3] = 0;
   out_1903674002302263901[4] = 0;
   out_1903674002302263901[5] = 0;
   out_1903674002302263901[6] = 0;
   out_1903674002302263901[7] = 0;
   out_1903674002302263901[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_6280992013641613331) {
  err_fun(nom_x, delta_x, out_6280992013641613331);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7070051882147892355) {
  inv_err_fun(nom_x, true_x, out_7070051882147892355);
}
void car_H_mod_fun(double *state, double *out_7380527807372010906) {
  H_mod_fun(state, out_7380527807372010906);
}
void car_f_fun(double *state, double dt, double *out_6083295923350312382) {
  f_fun(state,  dt, out_6083295923350312382);
}
void car_F_fun(double *state, double dt, double *out_7234405902953304843) {
  F_fun(state,  dt, out_7234405902953304843);
}
void car_h_25(double *state, double *unused, double *out_1871314369284315990) {
  h_25(state, unused, out_1871314369284315990);
}
void car_H_25(double *state, double *unused, double *out_6271385423409671601) {
  H_25(state, unused, out_6271385423409671601);
}
void car_h_24(double *state, double *unused, double *out_4554521396056145628) {
  h_24(state, unused, out_4554521396056145628);
}
void car_H_24(double *state, double *unused, double *out_4098735824404172035) {
  H_24(state, unused, out_4098735824404172035);
}
void car_h_30(double *state, double *unused, double *out_1714127700933186845) {
  h_30(state, unused, out_1714127700933186845);
}
void car_H_30(double *state, double *unused, double *out_8789718381916920228) {
  H_30(state, unused, out_8789718381916920228);
}
void car_h_26(double *state, double *unused, double *out_8113169218347035991) {
  h_26(state, unused, out_8113169218347035991);
}
void car_H_26(double *state, double *unused, double *out_2529882104535615377) {
  H_26(state, unused, out_2529882104535615377);
}
void car_h_27(double *state, double *unused, double *out_7110654802722524775) {
  h_27(state, unused, out_7110654802722524775);
}
void car_H_27(double *state, double *unused, double *out_6614955070116495317) {
  H_27(state, unused, out_6614955070116495317);
}
void car_h_29(double *state, double *unused, double *out_770617848031524410) {
  h_29(state, unused, out_770617848031524410);
}
void car_H_29(double *state, double *unused, double *out_9146794347478239204) {
  H_29(state, unused, out_9146794347478239204);
}
void car_h_28(double *state, double *unused, double *out_2108981535061679130) {
  h_28(state, unused, out_2108981535061679130);
}
void car_H_28(double *state, double *unused, double *out_4217550709161781838) {
  H_28(state, unused, out_4217550709161781838);
}
void car_h_31(double *state, double *unused, double *out_7785047395955064363) {
  h_31(state, unused, out_7785047395955064363);
}
void car_H_31(double *state, double *unused, double *out_1903674002302263901) {
  H_31(state, unused, out_1903674002302263901);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
