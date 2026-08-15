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
void car_err_fun(double *nom_x, double *delta_x, double *out_9079855518504475081);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1900044958656992730);
void car_H_mod_fun(double *state, double *out_8745830988861885773);
void car_f_fun(double *state, double dt, double *out_3575005069037443994);
void car_F_fun(double *state, double dt, double *out_8307510933637307448);
void car_h_25(double *state, double *unused, double *out_6120013002342940348);
void car_H_25(double *state, double *unused, double *out_507724858935428606);
void car_h_24(double *state, double *unused, double *out_3643429552749560204);
void car_H_24(double *state, double *unused, double *out_8267167548876742288);
void car_h_30(double *state, double *unused, double *out_6412998044695862895);
void car_H_30(double *state, double *unused, double *out_7424415200427045361);
void car_h_26(double *state, double *unused, double *out_1411324777035017250);
void car_H_26(double *state, double *unused, double *out_3233778459938627618);
void car_h_27(double *state, double *unused, double *out_1118339734682094703);
void car_H_27(double *state, double *unused, double *out_5249651888626620450);
void car_h_29(double *state, double *unused, double *out_2568971127310977870);
void car_H_29(double *state, double *unused, double *out_3536289161757069417);
void car_h_28(double *state, double *unused, double *out_6531005438629239639);
void car_H_28(double *state, double *unused, double *out_1546109855312461157);
void car_h_31(double *state, double *unused, double *out_8689281543112588918);
void car_H_31(double *state, double *unused, double *out_538370820812389034);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}