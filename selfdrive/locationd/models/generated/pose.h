#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_2903892006356229305);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_1008699991617682266);
void pose_H_mod_fun(double *state, double *out_8262236992047738224);
void pose_f_fun(double *state, double dt, double *out_5777010685098588684);
void pose_F_fun(double *state, double dt, double *out_1785538860011231665);
void pose_h_4(double *state, double *unused, double *out_1907516981769351839);
void pose_H_4(double *state, double *unused, double *out_6412699871817905820);
void pose_h_10(double *state, double *unused, double *out_5032815439302672244);
void pose_H_10(double *state, double *unused, double *out_475369215550419093);
void pose_h_13(double *state, double *unused, double *out_2054920899761121640);
void pose_H_13(double *state, double *unused, double *out_8200288738589121772);
void pose_h_14(double *state, double *unused, double *out_3615713150161481771);
void pose_H_14(double *state, double *unused, double *out_8951255769596273500);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}