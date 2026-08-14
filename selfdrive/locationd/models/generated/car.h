#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_6280992013641613331);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7070051882147892355);
void car_H_mod_fun(double *state, double *out_7380527807372010906);
void car_f_fun(double *state, double dt, double *out_6083295923350312382);
void car_F_fun(double *state, double dt, double *out_7234405902953304843);
void car_h_25(double *state, double *unused, double *out_1871314369284315990);
void car_H_25(double *state, double *unused, double *out_6271385423409671601);
void car_h_24(double *state, double *unused, double *out_4554521396056145628);
void car_H_24(double *state, double *unused, double *out_4098735824404172035);
void car_h_30(double *state, double *unused, double *out_1714127700933186845);
void car_H_30(double *state, double *unused, double *out_8789718381916920228);
void car_h_26(double *state, double *unused, double *out_8113169218347035991);
void car_H_26(double *state, double *unused, double *out_2529882104535615377);
void car_h_27(double *state, double *unused, double *out_7110654802722524775);
void car_H_27(double *state, double *unused, double *out_6614955070116495317);
void car_h_29(double *state, double *unused, double *out_770617848031524410);
void car_H_29(double *state, double *unused, double *out_9146794347478239204);
void car_h_28(double *state, double *unused, double *out_2108981535061679130);
void car_H_28(double *state, double *unused, double *out_4217550709161781838);
void car_h_31(double *state, double *unused, double *out_7785047395955064363);
void car_H_31(double *state, double *unused, double *out_1903674002302263901);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}