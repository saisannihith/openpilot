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
void err_fun(double *nom_x, double *delta_x, double *out_9079855518504475081) {
   out_9079855518504475081[0] = delta_x[0] + nom_x[0];
   out_9079855518504475081[1] = delta_x[1] + nom_x[1];
   out_9079855518504475081[2] = delta_x[2] + nom_x[2];
   out_9079855518504475081[3] = delta_x[3] + nom_x[3];
   out_9079855518504475081[4] = delta_x[4] + nom_x[4];
   out_9079855518504475081[5] = delta_x[5] + nom_x[5];
   out_9079855518504475081[6] = delta_x[6] + nom_x[6];
   out_9079855518504475081[7] = delta_x[7] + nom_x[7];
   out_9079855518504475081[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1900044958656992730) {
   out_1900044958656992730[0] = -nom_x[0] + true_x[0];
   out_1900044958656992730[1] = -nom_x[1] + true_x[1];
   out_1900044958656992730[2] = -nom_x[2] + true_x[2];
   out_1900044958656992730[3] = -nom_x[3] + true_x[3];
   out_1900044958656992730[4] = -nom_x[4] + true_x[4];
   out_1900044958656992730[5] = -nom_x[5] + true_x[5];
   out_1900044958656992730[6] = -nom_x[6] + true_x[6];
   out_1900044958656992730[7] = -nom_x[7] + true_x[7];
   out_1900044958656992730[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_8745830988861885773) {
   out_8745830988861885773[0] = 1.0;
   out_8745830988861885773[1] = 0.0;
   out_8745830988861885773[2] = 0.0;
   out_8745830988861885773[3] = 0.0;
   out_8745830988861885773[4] = 0.0;
   out_8745830988861885773[5] = 0.0;
   out_8745830988861885773[6] = 0.0;
   out_8745830988861885773[7] = 0.0;
   out_8745830988861885773[8] = 0.0;
   out_8745830988861885773[9] = 0.0;
   out_8745830988861885773[10] = 1.0;
   out_8745830988861885773[11] = 0.0;
   out_8745830988861885773[12] = 0.0;
   out_8745830988861885773[13] = 0.0;
   out_8745830988861885773[14] = 0.0;
   out_8745830988861885773[15] = 0.0;
   out_8745830988861885773[16] = 0.0;
   out_8745830988861885773[17] = 0.0;
   out_8745830988861885773[18] = 0.0;
   out_8745830988861885773[19] = 0.0;
   out_8745830988861885773[20] = 1.0;
   out_8745830988861885773[21] = 0.0;
   out_8745830988861885773[22] = 0.0;
   out_8745830988861885773[23] = 0.0;
   out_8745830988861885773[24] = 0.0;
   out_8745830988861885773[25] = 0.0;
   out_8745830988861885773[26] = 0.0;
   out_8745830988861885773[27] = 0.0;
   out_8745830988861885773[28] = 0.0;
   out_8745830988861885773[29] = 0.0;
   out_8745830988861885773[30] = 1.0;
   out_8745830988861885773[31] = 0.0;
   out_8745830988861885773[32] = 0.0;
   out_8745830988861885773[33] = 0.0;
   out_8745830988861885773[34] = 0.0;
   out_8745830988861885773[35] = 0.0;
   out_8745830988861885773[36] = 0.0;
   out_8745830988861885773[37] = 0.0;
   out_8745830988861885773[38] = 0.0;
   out_8745830988861885773[39] = 0.0;
   out_8745830988861885773[40] = 1.0;
   out_8745830988861885773[41] = 0.0;
   out_8745830988861885773[42] = 0.0;
   out_8745830988861885773[43] = 0.0;
   out_8745830988861885773[44] = 0.0;
   out_8745830988861885773[45] = 0.0;
   out_8745830988861885773[46] = 0.0;
   out_8745830988861885773[47] = 0.0;
   out_8745830988861885773[48] = 0.0;
   out_8745830988861885773[49] = 0.0;
   out_8745830988861885773[50] = 1.0;
   out_8745830988861885773[51] = 0.0;
   out_8745830988861885773[52] = 0.0;
   out_8745830988861885773[53] = 0.0;
   out_8745830988861885773[54] = 0.0;
   out_8745830988861885773[55] = 0.0;
   out_8745830988861885773[56] = 0.0;
   out_8745830988861885773[57] = 0.0;
   out_8745830988861885773[58] = 0.0;
   out_8745830988861885773[59] = 0.0;
   out_8745830988861885773[60] = 1.0;
   out_8745830988861885773[61] = 0.0;
   out_8745830988861885773[62] = 0.0;
   out_8745830988861885773[63] = 0.0;
   out_8745830988861885773[64] = 0.0;
   out_8745830988861885773[65] = 0.0;
   out_8745830988861885773[66] = 0.0;
   out_8745830988861885773[67] = 0.0;
   out_8745830988861885773[68] = 0.0;
   out_8745830988861885773[69] = 0.0;
   out_8745830988861885773[70] = 1.0;
   out_8745830988861885773[71] = 0.0;
   out_8745830988861885773[72] = 0.0;
   out_8745830988861885773[73] = 0.0;
   out_8745830988861885773[74] = 0.0;
   out_8745830988861885773[75] = 0.0;
   out_8745830988861885773[76] = 0.0;
   out_8745830988861885773[77] = 0.0;
   out_8745830988861885773[78] = 0.0;
   out_8745830988861885773[79] = 0.0;
   out_8745830988861885773[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3575005069037443994) {
   out_3575005069037443994[0] = state[0];
   out_3575005069037443994[1] = state[1];
   out_3575005069037443994[2] = state[2];
   out_3575005069037443994[3] = state[3];
   out_3575005069037443994[4] = state[4];
   out_3575005069037443994[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3575005069037443994[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3575005069037443994[7] = state[7];
   out_3575005069037443994[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8307510933637307448) {
   out_8307510933637307448[0] = 1;
   out_8307510933637307448[1] = 0;
   out_8307510933637307448[2] = 0;
   out_8307510933637307448[3] = 0;
   out_8307510933637307448[4] = 0;
   out_8307510933637307448[5] = 0;
   out_8307510933637307448[6] = 0;
   out_8307510933637307448[7] = 0;
   out_8307510933637307448[8] = 0;
   out_8307510933637307448[9] = 0;
   out_8307510933637307448[10] = 1;
   out_8307510933637307448[11] = 0;
   out_8307510933637307448[12] = 0;
   out_8307510933637307448[13] = 0;
   out_8307510933637307448[14] = 0;
   out_8307510933637307448[15] = 0;
   out_8307510933637307448[16] = 0;
   out_8307510933637307448[17] = 0;
   out_8307510933637307448[18] = 0;
   out_8307510933637307448[19] = 0;
   out_8307510933637307448[20] = 1;
   out_8307510933637307448[21] = 0;
   out_8307510933637307448[22] = 0;
   out_8307510933637307448[23] = 0;
   out_8307510933637307448[24] = 0;
   out_8307510933637307448[25] = 0;
   out_8307510933637307448[26] = 0;
   out_8307510933637307448[27] = 0;
   out_8307510933637307448[28] = 0;
   out_8307510933637307448[29] = 0;
   out_8307510933637307448[30] = 1;
   out_8307510933637307448[31] = 0;
   out_8307510933637307448[32] = 0;
   out_8307510933637307448[33] = 0;
   out_8307510933637307448[34] = 0;
   out_8307510933637307448[35] = 0;
   out_8307510933637307448[36] = 0;
   out_8307510933637307448[37] = 0;
   out_8307510933637307448[38] = 0;
   out_8307510933637307448[39] = 0;
   out_8307510933637307448[40] = 1;
   out_8307510933637307448[41] = 0;
   out_8307510933637307448[42] = 0;
   out_8307510933637307448[43] = 0;
   out_8307510933637307448[44] = 0;
   out_8307510933637307448[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8307510933637307448[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8307510933637307448[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8307510933637307448[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8307510933637307448[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8307510933637307448[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8307510933637307448[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8307510933637307448[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8307510933637307448[53] = -9.8100000000000005*dt;
   out_8307510933637307448[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8307510933637307448[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8307510933637307448[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8307510933637307448[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8307510933637307448[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8307510933637307448[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8307510933637307448[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8307510933637307448[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8307510933637307448[62] = 0;
   out_8307510933637307448[63] = 0;
   out_8307510933637307448[64] = 0;
   out_8307510933637307448[65] = 0;
   out_8307510933637307448[66] = 0;
   out_8307510933637307448[67] = 0;
   out_8307510933637307448[68] = 0;
   out_8307510933637307448[69] = 0;
   out_8307510933637307448[70] = 1;
   out_8307510933637307448[71] = 0;
   out_8307510933637307448[72] = 0;
   out_8307510933637307448[73] = 0;
   out_8307510933637307448[74] = 0;
   out_8307510933637307448[75] = 0;
   out_8307510933637307448[76] = 0;
   out_8307510933637307448[77] = 0;
   out_8307510933637307448[78] = 0;
   out_8307510933637307448[79] = 0;
   out_8307510933637307448[80] = 1;
}
void h_25(double *state, double *unused, double *out_6120013002342940348) {
   out_6120013002342940348[0] = state[6];
}
void H_25(double *state, double *unused, double *out_507724858935428606) {
   out_507724858935428606[0] = 0;
   out_507724858935428606[1] = 0;
   out_507724858935428606[2] = 0;
   out_507724858935428606[3] = 0;
   out_507724858935428606[4] = 0;
   out_507724858935428606[5] = 0;
   out_507724858935428606[6] = 1;
   out_507724858935428606[7] = 0;
   out_507724858935428606[8] = 0;
}
void h_24(double *state, double *unused, double *out_3643429552749560204) {
   out_3643429552749560204[0] = state[4];
   out_3643429552749560204[1] = state[5];
}
void H_24(double *state, double *unused, double *out_8267167548876742288) {
   out_8267167548876742288[0] = 0;
   out_8267167548876742288[1] = 0;
   out_8267167548876742288[2] = 0;
   out_8267167548876742288[3] = 0;
   out_8267167548876742288[4] = 1;
   out_8267167548876742288[5] = 0;
   out_8267167548876742288[6] = 0;
   out_8267167548876742288[7] = 0;
   out_8267167548876742288[8] = 0;
   out_8267167548876742288[9] = 0;
   out_8267167548876742288[10] = 0;
   out_8267167548876742288[11] = 0;
   out_8267167548876742288[12] = 0;
   out_8267167548876742288[13] = 0;
   out_8267167548876742288[14] = 1;
   out_8267167548876742288[15] = 0;
   out_8267167548876742288[16] = 0;
   out_8267167548876742288[17] = 0;
}
void h_30(double *state, double *unused, double *out_6412998044695862895) {
   out_6412998044695862895[0] = state[4];
}
void H_30(double *state, double *unused, double *out_7424415200427045361) {
   out_7424415200427045361[0] = 0;
   out_7424415200427045361[1] = 0;
   out_7424415200427045361[2] = 0;
   out_7424415200427045361[3] = 0;
   out_7424415200427045361[4] = 1;
   out_7424415200427045361[5] = 0;
   out_7424415200427045361[6] = 0;
   out_7424415200427045361[7] = 0;
   out_7424415200427045361[8] = 0;
}
void h_26(double *state, double *unused, double *out_1411324777035017250) {
   out_1411324777035017250[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3233778459938627618) {
   out_3233778459938627618[0] = 0;
   out_3233778459938627618[1] = 0;
   out_3233778459938627618[2] = 0;
   out_3233778459938627618[3] = 0;
   out_3233778459938627618[4] = 0;
   out_3233778459938627618[5] = 0;
   out_3233778459938627618[6] = 0;
   out_3233778459938627618[7] = 1;
   out_3233778459938627618[8] = 0;
}
void h_27(double *state, double *unused, double *out_1118339734682094703) {
   out_1118339734682094703[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5249651888626620450) {
   out_5249651888626620450[0] = 0;
   out_5249651888626620450[1] = 0;
   out_5249651888626620450[2] = 0;
   out_5249651888626620450[3] = 1;
   out_5249651888626620450[4] = 0;
   out_5249651888626620450[5] = 0;
   out_5249651888626620450[6] = 0;
   out_5249651888626620450[7] = 0;
   out_5249651888626620450[8] = 0;
}
void h_29(double *state, double *unused, double *out_2568971127310977870) {
   out_2568971127310977870[0] = state[1];
}
void H_29(double *state, double *unused, double *out_3536289161757069417) {
   out_3536289161757069417[0] = 0;
   out_3536289161757069417[1] = 1;
   out_3536289161757069417[2] = 0;
   out_3536289161757069417[3] = 0;
   out_3536289161757069417[4] = 0;
   out_3536289161757069417[5] = 0;
   out_3536289161757069417[6] = 0;
   out_3536289161757069417[7] = 0;
   out_3536289161757069417[8] = 0;
}
void h_28(double *state, double *unused, double *out_6531005438629239639) {
   out_6531005438629239639[0] = state[0];
}
void H_28(double *state, double *unused, double *out_1546109855312461157) {
   out_1546109855312461157[0] = 1;
   out_1546109855312461157[1] = 0;
   out_1546109855312461157[2] = 0;
   out_1546109855312461157[3] = 0;
   out_1546109855312461157[4] = 0;
   out_1546109855312461157[5] = 0;
   out_1546109855312461157[6] = 0;
   out_1546109855312461157[7] = 0;
   out_1546109855312461157[8] = 0;
}
void h_31(double *state, double *unused, double *out_8689281543112588918) {
   out_8689281543112588918[0] = state[8];
}
void H_31(double *state, double *unused, double *out_538370820812389034) {
   out_538370820812389034[0] = 0;
   out_538370820812389034[1] = 0;
   out_538370820812389034[2] = 0;
   out_538370820812389034[3] = 0;
   out_538370820812389034[4] = 0;
   out_538370820812389034[5] = 0;
   out_538370820812389034[6] = 0;
   out_538370820812389034[7] = 0;
   out_538370820812389034[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_9079855518504475081) {
  err_fun(nom_x, delta_x, out_9079855518504475081);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1900044958656992730) {
  inv_err_fun(nom_x, true_x, out_1900044958656992730);
}
void car_H_mod_fun(double *state, double *out_8745830988861885773) {
  H_mod_fun(state, out_8745830988861885773);
}
void car_f_fun(double *state, double dt, double *out_3575005069037443994) {
  f_fun(state,  dt, out_3575005069037443994);
}
void car_F_fun(double *state, double dt, double *out_8307510933637307448) {
  F_fun(state,  dt, out_8307510933637307448);
}
void car_h_25(double *state, double *unused, double *out_6120013002342940348) {
  h_25(state, unused, out_6120013002342940348);
}
void car_H_25(double *state, double *unused, double *out_507724858935428606) {
  H_25(state, unused, out_507724858935428606);
}
void car_h_24(double *state, double *unused, double *out_3643429552749560204) {
  h_24(state, unused, out_3643429552749560204);
}
void car_H_24(double *state, double *unused, double *out_8267167548876742288) {
  H_24(state, unused, out_8267167548876742288);
}
void car_h_30(double *state, double *unused, double *out_6412998044695862895) {
  h_30(state, unused, out_6412998044695862895);
}
void car_H_30(double *state, double *unused, double *out_7424415200427045361) {
  H_30(state, unused, out_7424415200427045361);
}
void car_h_26(double *state, double *unused, double *out_1411324777035017250) {
  h_26(state, unused, out_1411324777035017250);
}
void car_H_26(double *state, double *unused, double *out_3233778459938627618) {
  H_26(state, unused, out_3233778459938627618);
}
void car_h_27(double *state, double *unused, double *out_1118339734682094703) {
  h_27(state, unused, out_1118339734682094703);
}
void car_H_27(double *state, double *unused, double *out_5249651888626620450) {
  H_27(state, unused, out_5249651888626620450);
}
void car_h_29(double *state, double *unused, double *out_2568971127310977870) {
  h_29(state, unused, out_2568971127310977870);
}
void car_H_29(double *state, double *unused, double *out_3536289161757069417) {
  H_29(state, unused, out_3536289161757069417);
}
void car_h_28(double *state, double *unused, double *out_6531005438629239639) {
  h_28(state, unused, out_6531005438629239639);
}
void car_H_28(double *state, double *unused, double *out_1546109855312461157) {
  H_28(state, unused, out_1546109855312461157);
}
void car_h_31(double *state, double *unused, double *out_8689281543112588918) {
  h_31(state, unused, out_8689281543112588918);
}
void car_H_31(double *state, double *unused, double *out_538370820812389034) {
  H_31(state, unused, out_538370820812389034);
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
