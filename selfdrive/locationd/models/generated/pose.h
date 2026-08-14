#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_7868974326132984287);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_1909625330139833135);
void pose_H_mod_fun(double *state, double *out_839159570712955856);
void pose_f_fun(double *state, double dt, double *out_6870720994996901920);
void pose_F_fun(double *state, double dt, double *out_8965886230000449482);
void pose_h_4(double *state, double *unused, double *out_997896127340448425);
void pose_H_4(double *state, double *unused, double *out_4113381649503905109);
void pose_h_10(double *state, double *unused, double *out_8849325349390195320);
void pose_H_10(double *state, double *unused, double *out_3026541354882437243);
void pose_h_13(double *state, double *unused, double *out_8579573593214207751);
void pose_H_13(double *state, double *unused, double *out_901107824171572308);
void pose_h_14(double *state, double *unused, double *out_4065584171885884559);
void pose_H_14(double *state, double *unused, double *out_150140793164420580);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}