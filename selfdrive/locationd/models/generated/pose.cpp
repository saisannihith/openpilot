#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2903892006356229305) {
   out_2903892006356229305[0] = delta_x[0] + nom_x[0];
   out_2903892006356229305[1] = delta_x[1] + nom_x[1];
   out_2903892006356229305[2] = delta_x[2] + nom_x[2];
   out_2903892006356229305[3] = delta_x[3] + nom_x[3];
   out_2903892006356229305[4] = delta_x[4] + nom_x[4];
   out_2903892006356229305[5] = delta_x[5] + nom_x[5];
   out_2903892006356229305[6] = delta_x[6] + nom_x[6];
   out_2903892006356229305[7] = delta_x[7] + nom_x[7];
   out_2903892006356229305[8] = delta_x[8] + nom_x[8];
   out_2903892006356229305[9] = delta_x[9] + nom_x[9];
   out_2903892006356229305[10] = delta_x[10] + nom_x[10];
   out_2903892006356229305[11] = delta_x[11] + nom_x[11];
   out_2903892006356229305[12] = delta_x[12] + nom_x[12];
   out_2903892006356229305[13] = delta_x[13] + nom_x[13];
   out_2903892006356229305[14] = delta_x[14] + nom_x[14];
   out_2903892006356229305[15] = delta_x[15] + nom_x[15];
   out_2903892006356229305[16] = delta_x[16] + nom_x[16];
   out_2903892006356229305[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1008699991617682266) {
   out_1008699991617682266[0] = -nom_x[0] + true_x[0];
   out_1008699991617682266[1] = -nom_x[1] + true_x[1];
   out_1008699991617682266[2] = -nom_x[2] + true_x[2];
   out_1008699991617682266[3] = -nom_x[3] + true_x[3];
   out_1008699991617682266[4] = -nom_x[4] + true_x[4];
   out_1008699991617682266[5] = -nom_x[5] + true_x[5];
   out_1008699991617682266[6] = -nom_x[6] + true_x[6];
   out_1008699991617682266[7] = -nom_x[7] + true_x[7];
   out_1008699991617682266[8] = -nom_x[8] + true_x[8];
   out_1008699991617682266[9] = -nom_x[9] + true_x[9];
   out_1008699991617682266[10] = -nom_x[10] + true_x[10];
   out_1008699991617682266[11] = -nom_x[11] + true_x[11];
   out_1008699991617682266[12] = -nom_x[12] + true_x[12];
   out_1008699991617682266[13] = -nom_x[13] + true_x[13];
   out_1008699991617682266[14] = -nom_x[14] + true_x[14];
   out_1008699991617682266[15] = -nom_x[15] + true_x[15];
   out_1008699991617682266[16] = -nom_x[16] + true_x[16];
   out_1008699991617682266[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_8262236992047738224) {
   out_8262236992047738224[0] = 1.0;
   out_8262236992047738224[1] = 0.0;
   out_8262236992047738224[2] = 0.0;
   out_8262236992047738224[3] = 0.0;
   out_8262236992047738224[4] = 0.0;
   out_8262236992047738224[5] = 0.0;
   out_8262236992047738224[6] = 0.0;
   out_8262236992047738224[7] = 0.0;
   out_8262236992047738224[8] = 0.0;
   out_8262236992047738224[9] = 0.0;
   out_8262236992047738224[10] = 0.0;
   out_8262236992047738224[11] = 0.0;
   out_8262236992047738224[12] = 0.0;
   out_8262236992047738224[13] = 0.0;
   out_8262236992047738224[14] = 0.0;
   out_8262236992047738224[15] = 0.0;
   out_8262236992047738224[16] = 0.0;
   out_8262236992047738224[17] = 0.0;
   out_8262236992047738224[18] = 0.0;
   out_8262236992047738224[19] = 1.0;
   out_8262236992047738224[20] = 0.0;
   out_8262236992047738224[21] = 0.0;
   out_8262236992047738224[22] = 0.0;
   out_8262236992047738224[23] = 0.0;
   out_8262236992047738224[24] = 0.0;
   out_8262236992047738224[25] = 0.0;
   out_8262236992047738224[26] = 0.0;
   out_8262236992047738224[27] = 0.0;
   out_8262236992047738224[28] = 0.0;
   out_8262236992047738224[29] = 0.0;
   out_8262236992047738224[30] = 0.0;
   out_8262236992047738224[31] = 0.0;
   out_8262236992047738224[32] = 0.0;
   out_8262236992047738224[33] = 0.0;
   out_8262236992047738224[34] = 0.0;
   out_8262236992047738224[35] = 0.0;
   out_8262236992047738224[36] = 0.0;
   out_8262236992047738224[37] = 0.0;
   out_8262236992047738224[38] = 1.0;
   out_8262236992047738224[39] = 0.0;
   out_8262236992047738224[40] = 0.0;
   out_8262236992047738224[41] = 0.0;
   out_8262236992047738224[42] = 0.0;
   out_8262236992047738224[43] = 0.0;
   out_8262236992047738224[44] = 0.0;
   out_8262236992047738224[45] = 0.0;
   out_8262236992047738224[46] = 0.0;
   out_8262236992047738224[47] = 0.0;
   out_8262236992047738224[48] = 0.0;
   out_8262236992047738224[49] = 0.0;
   out_8262236992047738224[50] = 0.0;
   out_8262236992047738224[51] = 0.0;
   out_8262236992047738224[52] = 0.0;
   out_8262236992047738224[53] = 0.0;
   out_8262236992047738224[54] = 0.0;
   out_8262236992047738224[55] = 0.0;
   out_8262236992047738224[56] = 0.0;
   out_8262236992047738224[57] = 1.0;
   out_8262236992047738224[58] = 0.0;
   out_8262236992047738224[59] = 0.0;
   out_8262236992047738224[60] = 0.0;
   out_8262236992047738224[61] = 0.0;
   out_8262236992047738224[62] = 0.0;
   out_8262236992047738224[63] = 0.0;
   out_8262236992047738224[64] = 0.0;
   out_8262236992047738224[65] = 0.0;
   out_8262236992047738224[66] = 0.0;
   out_8262236992047738224[67] = 0.0;
   out_8262236992047738224[68] = 0.0;
   out_8262236992047738224[69] = 0.0;
   out_8262236992047738224[70] = 0.0;
   out_8262236992047738224[71] = 0.0;
   out_8262236992047738224[72] = 0.0;
   out_8262236992047738224[73] = 0.0;
   out_8262236992047738224[74] = 0.0;
   out_8262236992047738224[75] = 0.0;
   out_8262236992047738224[76] = 1.0;
   out_8262236992047738224[77] = 0.0;
   out_8262236992047738224[78] = 0.0;
   out_8262236992047738224[79] = 0.0;
   out_8262236992047738224[80] = 0.0;
   out_8262236992047738224[81] = 0.0;
   out_8262236992047738224[82] = 0.0;
   out_8262236992047738224[83] = 0.0;
   out_8262236992047738224[84] = 0.0;
   out_8262236992047738224[85] = 0.0;
   out_8262236992047738224[86] = 0.0;
   out_8262236992047738224[87] = 0.0;
   out_8262236992047738224[88] = 0.0;
   out_8262236992047738224[89] = 0.0;
   out_8262236992047738224[90] = 0.0;
   out_8262236992047738224[91] = 0.0;
   out_8262236992047738224[92] = 0.0;
   out_8262236992047738224[93] = 0.0;
   out_8262236992047738224[94] = 0.0;
   out_8262236992047738224[95] = 1.0;
   out_8262236992047738224[96] = 0.0;
   out_8262236992047738224[97] = 0.0;
   out_8262236992047738224[98] = 0.0;
   out_8262236992047738224[99] = 0.0;
   out_8262236992047738224[100] = 0.0;
   out_8262236992047738224[101] = 0.0;
   out_8262236992047738224[102] = 0.0;
   out_8262236992047738224[103] = 0.0;
   out_8262236992047738224[104] = 0.0;
   out_8262236992047738224[105] = 0.0;
   out_8262236992047738224[106] = 0.0;
   out_8262236992047738224[107] = 0.0;
   out_8262236992047738224[108] = 0.0;
   out_8262236992047738224[109] = 0.0;
   out_8262236992047738224[110] = 0.0;
   out_8262236992047738224[111] = 0.0;
   out_8262236992047738224[112] = 0.0;
   out_8262236992047738224[113] = 0.0;
   out_8262236992047738224[114] = 1.0;
   out_8262236992047738224[115] = 0.0;
   out_8262236992047738224[116] = 0.0;
   out_8262236992047738224[117] = 0.0;
   out_8262236992047738224[118] = 0.0;
   out_8262236992047738224[119] = 0.0;
   out_8262236992047738224[120] = 0.0;
   out_8262236992047738224[121] = 0.0;
   out_8262236992047738224[122] = 0.0;
   out_8262236992047738224[123] = 0.0;
   out_8262236992047738224[124] = 0.0;
   out_8262236992047738224[125] = 0.0;
   out_8262236992047738224[126] = 0.0;
   out_8262236992047738224[127] = 0.0;
   out_8262236992047738224[128] = 0.0;
   out_8262236992047738224[129] = 0.0;
   out_8262236992047738224[130] = 0.0;
   out_8262236992047738224[131] = 0.0;
   out_8262236992047738224[132] = 0.0;
   out_8262236992047738224[133] = 1.0;
   out_8262236992047738224[134] = 0.0;
   out_8262236992047738224[135] = 0.0;
   out_8262236992047738224[136] = 0.0;
   out_8262236992047738224[137] = 0.0;
   out_8262236992047738224[138] = 0.0;
   out_8262236992047738224[139] = 0.0;
   out_8262236992047738224[140] = 0.0;
   out_8262236992047738224[141] = 0.0;
   out_8262236992047738224[142] = 0.0;
   out_8262236992047738224[143] = 0.0;
   out_8262236992047738224[144] = 0.0;
   out_8262236992047738224[145] = 0.0;
   out_8262236992047738224[146] = 0.0;
   out_8262236992047738224[147] = 0.0;
   out_8262236992047738224[148] = 0.0;
   out_8262236992047738224[149] = 0.0;
   out_8262236992047738224[150] = 0.0;
   out_8262236992047738224[151] = 0.0;
   out_8262236992047738224[152] = 1.0;
   out_8262236992047738224[153] = 0.0;
   out_8262236992047738224[154] = 0.0;
   out_8262236992047738224[155] = 0.0;
   out_8262236992047738224[156] = 0.0;
   out_8262236992047738224[157] = 0.0;
   out_8262236992047738224[158] = 0.0;
   out_8262236992047738224[159] = 0.0;
   out_8262236992047738224[160] = 0.0;
   out_8262236992047738224[161] = 0.0;
   out_8262236992047738224[162] = 0.0;
   out_8262236992047738224[163] = 0.0;
   out_8262236992047738224[164] = 0.0;
   out_8262236992047738224[165] = 0.0;
   out_8262236992047738224[166] = 0.0;
   out_8262236992047738224[167] = 0.0;
   out_8262236992047738224[168] = 0.0;
   out_8262236992047738224[169] = 0.0;
   out_8262236992047738224[170] = 0.0;
   out_8262236992047738224[171] = 1.0;
   out_8262236992047738224[172] = 0.0;
   out_8262236992047738224[173] = 0.0;
   out_8262236992047738224[174] = 0.0;
   out_8262236992047738224[175] = 0.0;
   out_8262236992047738224[176] = 0.0;
   out_8262236992047738224[177] = 0.0;
   out_8262236992047738224[178] = 0.0;
   out_8262236992047738224[179] = 0.0;
   out_8262236992047738224[180] = 0.0;
   out_8262236992047738224[181] = 0.0;
   out_8262236992047738224[182] = 0.0;
   out_8262236992047738224[183] = 0.0;
   out_8262236992047738224[184] = 0.0;
   out_8262236992047738224[185] = 0.0;
   out_8262236992047738224[186] = 0.0;
   out_8262236992047738224[187] = 0.0;
   out_8262236992047738224[188] = 0.0;
   out_8262236992047738224[189] = 0.0;
   out_8262236992047738224[190] = 1.0;
   out_8262236992047738224[191] = 0.0;
   out_8262236992047738224[192] = 0.0;
   out_8262236992047738224[193] = 0.0;
   out_8262236992047738224[194] = 0.0;
   out_8262236992047738224[195] = 0.0;
   out_8262236992047738224[196] = 0.0;
   out_8262236992047738224[197] = 0.0;
   out_8262236992047738224[198] = 0.0;
   out_8262236992047738224[199] = 0.0;
   out_8262236992047738224[200] = 0.0;
   out_8262236992047738224[201] = 0.0;
   out_8262236992047738224[202] = 0.0;
   out_8262236992047738224[203] = 0.0;
   out_8262236992047738224[204] = 0.0;
   out_8262236992047738224[205] = 0.0;
   out_8262236992047738224[206] = 0.0;
   out_8262236992047738224[207] = 0.0;
   out_8262236992047738224[208] = 0.0;
   out_8262236992047738224[209] = 1.0;
   out_8262236992047738224[210] = 0.0;
   out_8262236992047738224[211] = 0.0;
   out_8262236992047738224[212] = 0.0;
   out_8262236992047738224[213] = 0.0;
   out_8262236992047738224[214] = 0.0;
   out_8262236992047738224[215] = 0.0;
   out_8262236992047738224[216] = 0.0;
   out_8262236992047738224[217] = 0.0;
   out_8262236992047738224[218] = 0.0;
   out_8262236992047738224[219] = 0.0;
   out_8262236992047738224[220] = 0.0;
   out_8262236992047738224[221] = 0.0;
   out_8262236992047738224[222] = 0.0;
   out_8262236992047738224[223] = 0.0;
   out_8262236992047738224[224] = 0.0;
   out_8262236992047738224[225] = 0.0;
   out_8262236992047738224[226] = 0.0;
   out_8262236992047738224[227] = 0.0;
   out_8262236992047738224[228] = 1.0;
   out_8262236992047738224[229] = 0.0;
   out_8262236992047738224[230] = 0.0;
   out_8262236992047738224[231] = 0.0;
   out_8262236992047738224[232] = 0.0;
   out_8262236992047738224[233] = 0.0;
   out_8262236992047738224[234] = 0.0;
   out_8262236992047738224[235] = 0.0;
   out_8262236992047738224[236] = 0.0;
   out_8262236992047738224[237] = 0.0;
   out_8262236992047738224[238] = 0.0;
   out_8262236992047738224[239] = 0.0;
   out_8262236992047738224[240] = 0.0;
   out_8262236992047738224[241] = 0.0;
   out_8262236992047738224[242] = 0.0;
   out_8262236992047738224[243] = 0.0;
   out_8262236992047738224[244] = 0.0;
   out_8262236992047738224[245] = 0.0;
   out_8262236992047738224[246] = 0.0;
   out_8262236992047738224[247] = 1.0;
   out_8262236992047738224[248] = 0.0;
   out_8262236992047738224[249] = 0.0;
   out_8262236992047738224[250] = 0.0;
   out_8262236992047738224[251] = 0.0;
   out_8262236992047738224[252] = 0.0;
   out_8262236992047738224[253] = 0.0;
   out_8262236992047738224[254] = 0.0;
   out_8262236992047738224[255] = 0.0;
   out_8262236992047738224[256] = 0.0;
   out_8262236992047738224[257] = 0.0;
   out_8262236992047738224[258] = 0.0;
   out_8262236992047738224[259] = 0.0;
   out_8262236992047738224[260] = 0.0;
   out_8262236992047738224[261] = 0.0;
   out_8262236992047738224[262] = 0.0;
   out_8262236992047738224[263] = 0.0;
   out_8262236992047738224[264] = 0.0;
   out_8262236992047738224[265] = 0.0;
   out_8262236992047738224[266] = 1.0;
   out_8262236992047738224[267] = 0.0;
   out_8262236992047738224[268] = 0.0;
   out_8262236992047738224[269] = 0.0;
   out_8262236992047738224[270] = 0.0;
   out_8262236992047738224[271] = 0.0;
   out_8262236992047738224[272] = 0.0;
   out_8262236992047738224[273] = 0.0;
   out_8262236992047738224[274] = 0.0;
   out_8262236992047738224[275] = 0.0;
   out_8262236992047738224[276] = 0.0;
   out_8262236992047738224[277] = 0.0;
   out_8262236992047738224[278] = 0.0;
   out_8262236992047738224[279] = 0.0;
   out_8262236992047738224[280] = 0.0;
   out_8262236992047738224[281] = 0.0;
   out_8262236992047738224[282] = 0.0;
   out_8262236992047738224[283] = 0.0;
   out_8262236992047738224[284] = 0.0;
   out_8262236992047738224[285] = 1.0;
   out_8262236992047738224[286] = 0.0;
   out_8262236992047738224[287] = 0.0;
   out_8262236992047738224[288] = 0.0;
   out_8262236992047738224[289] = 0.0;
   out_8262236992047738224[290] = 0.0;
   out_8262236992047738224[291] = 0.0;
   out_8262236992047738224[292] = 0.0;
   out_8262236992047738224[293] = 0.0;
   out_8262236992047738224[294] = 0.0;
   out_8262236992047738224[295] = 0.0;
   out_8262236992047738224[296] = 0.0;
   out_8262236992047738224[297] = 0.0;
   out_8262236992047738224[298] = 0.0;
   out_8262236992047738224[299] = 0.0;
   out_8262236992047738224[300] = 0.0;
   out_8262236992047738224[301] = 0.0;
   out_8262236992047738224[302] = 0.0;
   out_8262236992047738224[303] = 0.0;
   out_8262236992047738224[304] = 1.0;
   out_8262236992047738224[305] = 0.0;
   out_8262236992047738224[306] = 0.0;
   out_8262236992047738224[307] = 0.0;
   out_8262236992047738224[308] = 0.0;
   out_8262236992047738224[309] = 0.0;
   out_8262236992047738224[310] = 0.0;
   out_8262236992047738224[311] = 0.0;
   out_8262236992047738224[312] = 0.0;
   out_8262236992047738224[313] = 0.0;
   out_8262236992047738224[314] = 0.0;
   out_8262236992047738224[315] = 0.0;
   out_8262236992047738224[316] = 0.0;
   out_8262236992047738224[317] = 0.0;
   out_8262236992047738224[318] = 0.0;
   out_8262236992047738224[319] = 0.0;
   out_8262236992047738224[320] = 0.0;
   out_8262236992047738224[321] = 0.0;
   out_8262236992047738224[322] = 0.0;
   out_8262236992047738224[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_5777010685098588684) {
   out_5777010685098588684[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_5777010685098588684[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_5777010685098588684[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_5777010685098588684[3] = dt*state[12] + state[3];
   out_5777010685098588684[4] = dt*state[13] + state[4];
   out_5777010685098588684[5] = dt*state[14] + state[5];
   out_5777010685098588684[6] = state[6];
   out_5777010685098588684[7] = state[7];
   out_5777010685098588684[8] = state[8];
   out_5777010685098588684[9] = state[9];
   out_5777010685098588684[10] = state[10];
   out_5777010685098588684[11] = state[11];
   out_5777010685098588684[12] = state[12];
   out_5777010685098588684[13] = state[13];
   out_5777010685098588684[14] = state[14];
   out_5777010685098588684[15] = state[15];
   out_5777010685098588684[16] = state[16];
   out_5777010685098588684[17] = state[17];
}
void F_fun(double *state, double dt, double *out_1785538860011231665) {
   out_1785538860011231665[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1785538860011231665[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1785538860011231665[2] = 0;
   out_1785538860011231665[3] = 0;
   out_1785538860011231665[4] = 0;
   out_1785538860011231665[5] = 0;
   out_1785538860011231665[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1785538860011231665[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1785538860011231665[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1785538860011231665[9] = 0;
   out_1785538860011231665[10] = 0;
   out_1785538860011231665[11] = 0;
   out_1785538860011231665[12] = 0;
   out_1785538860011231665[13] = 0;
   out_1785538860011231665[14] = 0;
   out_1785538860011231665[15] = 0;
   out_1785538860011231665[16] = 0;
   out_1785538860011231665[17] = 0;
   out_1785538860011231665[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1785538860011231665[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1785538860011231665[20] = 0;
   out_1785538860011231665[21] = 0;
   out_1785538860011231665[22] = 0;
   out_1785538860011231665[23] = 0;
   out_1785538860011231665[24] = 0;
   out_1785538860011231665[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1785538860011231665[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1785538860011231665[27] = 0;
   out_1785538860011231665[28] = 0;
   out_1785538860011231665[29] = 0;
   out_1785538860011231665[30] = 0;
   out_1785538860011231665[31] = 0;
   out_1785538860011231665[32] = 0;
   out_1785538860011231665[33] = 0;
   out_1785538860011231665[34] = 0;
   out_1785538860011231665[35] = 0;
   out_1785538860011231665[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1785538860011231665[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1785538860011231665[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1785538860011231665[39] = 0;
   out_1785538860011231665[40] = 0;
   out_1785538860011231665[41] = 0;
   out_1785538860011231665[42] = 0;
   out_1785538860011231665[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1785538860011231665[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1785538860011231665[45] = 0;
   out_1785538860011231665[46] = 0;
   out_1785538860011231665[47] = 0;
   out_1785538860011231665[48] = 0;
   out_1785538860011231665[49] = 0;
   out_1785538860011231665[50] = 0;
   out_1785538860011231665[51] = 0;
   out_1785538860011231665[52] = 0;
   out_1785538860011231665[53] = 0;
   out_1785538860011231665[54] = 0;
   out_1785538860011231665[55] = 0;
   out_1785538860011231665[56] = 0;
   out_1785538860011231665[57] = 1;
   out_1785538860011231665[58] = 0;
   out_1785538860011231665[59] = 0;
   out_1785538860011231665[60] = 0;
   out_1785538860011231665[61] = 0;
   out_1785538860011231665[62] = 0;
   out_1785538860011231665[63] = 0;
   out_1785538860011231665[64] = 0;
   out_1785538860011231665[65] = 0;
   out_1785538860011231665[66] = dt;
   out_1785538860011231665[67] = 0;
   out_1785538860011231665[68] = 0;
   out_1785538860011231665[69] = 0;
   out_1785538860011231665[70] = 0;
   out_1785538860011231665[71] = 0;
   out_1785538860011231665[72] = 0;
   out_1785538860011231665[73] = 0;
   out_1785538860011231665[74] = 0;
   out_1785538860011231665[75] = 0;
   out_1785538860011231665[76] = 1;
   out_1785538860011231665[77] = 0;
   out_1785538860011231665[78] = 0;
   out_1785538860011231665[79] = 0;
   out_1785538860011231665[80] = 0;
   out_1785538860011231665[81] = 0;
   out_1785538860011231665[82] = 0;
   out_1785538860011231665[83] = 0;
   out_1785538860011231665[84] = 0;
   out_1785538860011231665[85] = dt;
   out_1785538860011231665[86] = 0;
   out_1785538860011231665[87] = 0;
   out_1785538860011231665[88] = 0;
   out_1785538860011231665[89] = 0;
   out_1785538860011231665[90] = 0;
   out_1785538860011231665[91] = 0;
   out_1785538860011231665[92] = 0;
   out_1785538860011231665[93] = 0;
   out_1785538860011231665[94] = 0;
   out_1785538860011231665[95] = 1;
   out_1785538860011231665[96] = 0;
   out_1785538860011231665[97] = 0;
   out_1785538860011231665[98] = 0;
   out_1785538860011231665[99] = 0;
   out_1785538860011231665[100] = 0;
   out_1785538860011231665[101] = 0;
   out_1785538860011231665[102] = 0;
   out_1785538860011231665[103] = 0;
   out_1785538860011231665[104] = dt;
   out_1785538860011231665[105] = 0;
   out_1785538860011231665[106] = 0;
   out_1785538860011231665[107] = 0;
   out_1785538860011231665[108] = 0;
   out_1785538860011231665[109] = 0;
   out_1785538860011231665[110] = 0;
   out_1785538860011231665[111] = 0;
   out_1785538860011231665[112] = 0;
   out_1785538860011231665[113] = 0;
   out_1785538860011231665[114] = 1;
   out_1785538860011231665[115] = 0;
   out_1785538860011231665[116] = 0;
   out_1785538860011231665[117] = 0;
   out_1785538860011231665[118] = 0;
   out_1785538860011231665[119] = 0;
   out_1785538860011231665[120] = 0;
   out_1785538860011231665[121] = 0;
   out_1785538860011231665[122] = 0;
   out_1785538860011231665[123] = 0;
   out_1785538860011231665[124] = 0;
   out_1785538860011231665[125] = 0;
   out_1785538860011231665[126] = 0;
   out_1785538860011231665[127] = 0;
   out_1785538860011231665[128] = 0;
   out_1785538860011231665[129] = 0;
   out_1785538860011231665[130] = 0;
   out_1785538860011231665[131] = 0;
   out_1785538860011231665[132] = 0;
   out_1785538860011231665[133] = 1;
   out_1785538860011231665[134] = 0;
   out_1785538860011231665[135] = 0;
   out_1785538860011231665[136] = 0;
   out_1785538860011231665[137] = 0;
   out_1785538860011231665[138] = 0;
   out_1785538860011231665[139] = 0;
   out_1785538860011231665[140] = 0;
   out_1785538860011231665[141] = 0;
   out_1785538860011231665[142] = 0;
   out_1785538860011231665[143] = 0;
   out_1785538860011231665[144] = 0;
   out_1785538860011231665[145] = 0;
   out_1785538860011231665[146] = 0;
   out_1785538860011231665[147] = 0;
   out_1785538860011231665[148] = 0;
   out_1785538860011231665[149] = 0;
   out_1785538860011231665[150] = 0;
   out_1785538860011231665[151] = 0;
   out_1785538860011231665[152] = 1;
   out_1785538860011231665[153] = 0;
   out_1785538860011231665[154] = 0;
   out_1785538860011231665[155] = 0;
   out_1785538860011231665[156] = 0;
   out_1785538860011231665[157] = 0;
   out_1785538860011231665[158] = 0;
   out_1785538860011231665[159] = 0;
   out_1785538860011231665[160] = 0;
   out_1785538860011231665[161] = 0;
   out_1785538860011231665[162] = 0;
   out_1785538860011231665[163] = 0;
   out_1785538860011231665[164] = 0;
   out_1785538860011231665[165] = 0;
   out_1785538860011231665[166] = 0;
   out_1785538860011231665[167] = 0;
   out_1785538860011231665[168] = 0;
   out_1785538860011231665[169] = 0;
   out_1785538860011231665[170] = 0;
   out_1785538860011231665[171] = 1;
   out_1785538860011231665[172] = 0;
   out_1785538860011231665[173] = 0;
   out_1785538860011231665[174] = 0;
   out_1785538860011231665[175] = 0;
   out_1785538860011231665[176] = 0;
   out_1785538860011231665[177] = 0;
   out_1785538860011231665[178] = 0;
   out_1785538860011231665[179] = 0;
   out_1785538860011231665[180] = 0;
   out_1785538860011231665[181] = 0;
   out_1785538860011231665[182] = 0;
   out_1785538860011231665[183] = 0;
   out_1785538860011231665[184] = 0;
   out_1785538860011231665[185] = 0;
   out_1785538860011231665[186] = 0;
   out_1785538860011231665[187] = 0;
   out_1785538860011231665[188] = 0;
   out_1785538860011231665[189] = 0;
   out_1785538860011231665[190] = 1;
   out_1785538860011231665[191] = 0;
   out_1785538860011231665[192] = 0;
   out_1785538860011231665[193] = 0;
   out_1785538860011231665[194] = 0;
   out_1785538860011231665[195] = 0;
   out_1785538860011231665[196] = 0;
   out_1785538860011231665[197] = 0;
   out_1785538860011231665[198] = 0;
   out_1785538860011231665[199] = 0;
   out_1785538860011231665[200] = 0;
   out_1785538860011231665[201] = 0;
   out_1785538860011231665[202] = 0;
   out_1785538860011231665[203] = 0;
   out_1785538860011231665[204] = 0;
   out_1785538860011231665[205] = 0;
   out_1785538860011231665[206] = 0;
   out_1785538860011231665[207] = 0;
   out_1785538860011231665[208] = 0;
   out_1785538860011231665[209] = 1;
   out_1785538860011231665[210] = 0;
   out_1785538860011231665[211] = 0;
   out_1785538860011231665[212] = 0;
   out_1785538860011231665[213] = 0;
   out_1785538860011231665[214] = 0;
   out_1785538860011231665[215] = 0;
   out_1785538860011231665[216] = 0;
   out_1785538860011231665[217] = 0;
   out_1785538860011231665[218] = 0;
   out_1785538860011231665[219] = 0;
   out_1785538860011231665[220] = 0;
   out_1785538860011231665[221] = 0;
   out_1785538860011231665[222] = 0;
   out_1785538860011231665[223] = 0;
   out_1785538860011231665[224] = 0;
   out_1785538860011231665[225] = 0;
   out_1785538860011231665[226] = 0;
   out_1785538860011231665[227] = 0;
   out_1785538860011231665[228] = 1;
   out_1785538860011231665[229] = 0;
   out_1785538860011231665[230] = 0;
   out_1785538860011231665[231] = 0;
   out_1785538860011231665[232] = 0;
   out_1785538860011231665[233] = 0;
   out_1785538860011231665[234] = 0;
   out_1785538860011231665[235] = 0;
   out_1785538860011231665[236] = 0;
   out_1785538860011231665[237] = 0;
   out_1785538860011231665[238] = 0;
   out_1785538860011231665[239] = 0;
   out_1785538860011231665[240] = 0;
   out_1785538860011231665[241] = 0;
   out_1785538860011231665[242] = 0;
   out_1785538860011231665[243] = 0;
   out_1785538860011231665[244] = 0;
   out_1785538860011231665[245] = 0;
   out_1785538860011231665[246] = 0;
   out_1785538860011231665[247] = 1;
   out_1785538860011231665[248] = 0;
   out_1785538860011231665[249] = 0;
   out_1785538860011231665[250] = 0;
   out_1785538860011231665[251] = 0;
   out_1785538860011231665[252] = 0;
   out_1785538860011231665[253] = 0;
   out_1785538860011231665[254] = 0;
   out_1785538860011231665[255] = 0;
   out_1785538860011231665[256] = 0;
   out_1785538860011231665[257] = 0;
   out_1785538860011231665[258] = 0;
   out_1785538860011231665[259] = 0;
   out_1785538860011231665[260] = 0;
   out_1785538860011231665[261] = 0;
   out_1785538860011231665[262] = 0;
   out_1785538860011231665[263] = 0;
   out_1785538860011231665[264] = 0;
   out_1785538860011231665[265] = 0;
   out_1785538860011231665[266] = 1;
   out_1785538860011231665[267] = 0;
   out_1785538860011231665[268] = 0;
   out_1785538860011231665[269] = 0;
   out_1785538860011231665[270] = 0;
   out_1785538860011231665[271] = 0;
   out_1785538860011231665[272] = 0;
   out_1785538860011231665[273] = 0;
   out_1785538860011231665[274] = 0;
   out_1785538860011231665[275] = 0;
   out_1785538860011231665[276] = 0;
   out_1785538860011231665[277] = 0;
   out_1785538860011231665[278] = 0;
   out_1785538860011231665[279] = 0;
   out_1785538860011231665[280] = 0;
   out_1785538860011231665[281] = 0;
   out_1785538860011231665[282] = 0;
   out_1785538860011231665[283] = 0;
   out_1785538860011231665[284] = 0;
   out_1785538860011231665[285] = 1;
   out_1785538860011231665[286] = 0;
   out_1785538860011231665[287] = 0;
   out_1785538860011231665[288] = 0;
   out_1785538860011231665[289] = 0;
   out_1785538860011231665[290] = 0;
   out_1785538860011231665[291] = 0;
   out_1785538860011231665[292] = 0;
   out_1785538860011231665[293] = 0;
   out_1785538860011231665[294] = 0;
   out_1785538860011231665[295] = 0;
   out_1785538860011231665[296] = 0;
   out_1785538860011231665[297] = 0;
   out_1785538860011231665[298] = 0;
   out_1785538860011231665[299] = 0;
   out_1785538860011231665[300] = 0;
   out_1785538860011231665[301] = 0;
   out_1785538860011231665[302] = 0;
   out_1785538860011231665[303] = 0;
   out_1785538860011231665[304] = 1;
   out_1785538860011231665[305] = 0;
   out_1785538860011231665[306] = 0;
   out_1785538860011231665[307] = 0;
   out_1785538860011231665[308] = 0;
   out_1785538860011231665[309] = 0;
   out_1785538860011231665[310] = 0;
   out_1785538860011231665[311] = 0;
   out_1785538860011231665[312] = 0;
   out_1785538860011231665[313] = 0;
   out_1785538860011231665[314] = 0;
   out_1785538860011231665[315] = 0;
   out_1785538860011231665[316] = 0;
   out_1785538860011231665[317] = 0;
   out_1785538860011231665[318] = 0;
   out_1785538860011231665[319] = 0;
   out_1785538860011231665[320] = 0;
   out_1785538860011231665[321] = 0;
   out_1785538860011231665[322] = 0;
   out_1785538860011231665[323] = 1;
}
void h_4(double *state, double *unused, double *out_1907516981769351839) {
   out_1907516981769351839[0] = state[6] + state[9];
   out_1907516981769351839[1] = state[7] + state[10];
   out_1907516981769351839[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_6412699871817905820) {
   out_6412699871817905820[0] = 0;
   out_6412699871817905820[1] = 0;
   out_6412699871817905820[2] = 0;
   out_6412699871817905820[3] = 0;
   out_6412699871817905820[4] = 0;
   out_6412699871817905820[5] = 0;
   out_6412699871817905820[6] = 1;
   out_6412699871817905820[7] = 0;
   out_6412699871817905820[8] = 0;
   out_6412699871817905820[9] = 1;
   out_6412699871817905820[10] = 0;
   out_6412699871817905820[11] = 0;
   out_6412699871817905820[12] = 0;
   out_6412699871817905820[13] = 0;
   out_6412699871817905820[14] = 0;
   out_6412699871817905820[15] = 0;
   out_6412699871817905820[16] = 0;
   out_6412699871817905820[17] = 0;
   out_6412699871817905820[18] = 0;
   out_6412699871817905820[19] = 0;
   out_6412699871817905820[20] = 0;
   out_6412699871817905820[21] = 0;
   out_6412699871817905820[22] = 0;
   out_6412699871817905820[23] = 0;
   out_6412699871817905820[24] = 0;
   out_6412699871817905820[25] = 1;
   out_6412699871817905820[26] = 0;
   out_6412699871817905820[27] = 0;
   out_6412699871817905820[28] = 1;
   out_6412699871817905820[29] = 0;
   out_6412699871817905820[30] = 0;
   out_6412699871817905820[31] = 0;
   out_6412699871817905820[32] = 0;
   out_6412699871817905820[33] = 0;
   out_6412699871817905820[34] = 0;
   out_6412699871817905820[35] = 0;
   out_6412699871817905820[36] = 0;
   out_6412699871817905820[37] = 0;
   out_6412699871817905820[38] = 0;
   out_6412699871817905820[39] = 0;
   out_6412699871817905820[40] = 0;
   out_6412699871817905820[41] = 0;
   out_6412699871817905820[42] = 0;
   out_6412699871817905820[43] = 0;
   out_6412699871817905820[44] = 1;
   out_6412699871817905820[45] = 0;
   out_6412699871817905820[46] = 0;
   out_6412699871817905820[47] = 1;
   out_6412699871817905820[48] = 0;
   out_6412699871817905820[49] = 0;
   out_6412699871817905820[50] = 0;
   out_6412699871817905820[51] = 0;
   out_6412699871817905820[52] = 0;
   out_6412699871817905820[53] = 0;
}
void h_10(double *state, double *unused, double *out_5032815439302672244) {
   out_5032815439302672244[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_5032815439302672244[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_5032815439302672244[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_475369215550419093) {
   out_475369215550419093[0] = 0;
   out_475369215550419093[1] = 9.8100000000000005*cos(state[1]);
   out_475369215550419093[2] = 0;
   out_475369215550419093[3] = 0;
   out_475369215550419093[4] = -state[8];
   out_475369215550419093[5] = state[7];
   out_475369215550419093[6] = 0;
   out_475369215550419093[7] = state[5];
   out_475369215550419093[8] = -state[4];
   out_475369215550419093[9] = 0;
   out_475369215550419093[10] = 0;
   out_475369215550419093[11] = 0;
   out_475369215550419093[12] = 1;
   out_475369215550419093[13] = 0;
   out_475369215550419093[14] = 0;
   out_475369215550419093[15] = 1;
   out_475369215550419093[16] = 0;
   out_475369215550419093[17] = 0;
   out_475369215550419093[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_475369215550419093[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_475369215550419093[20] = 0;
   out_475369215550419093[21] = state[8];
   out_475369215550419093[22] = 0;
   out_475369215550419093[23] = -state[6];
   out_475369215550419093[24] = -state[5];
   out_475369215550419093[25] = 0;
   out_475369215550419093[26] = state[3];
   out_475369215550419093[27] = 0;
   out_475369215550419093[28] = 0;
   out_475369215550419093[29] = 0;
   out_475369215550419093[30] = 0;
   out_475369215550419093[31] = 1;
   out_475369215550419093[32] = 0;
   out_475369215550419093[33] = 0;
   out_475369215550419093[34] = 1;
   out_475369215550419093[35] = 0;
   out_475369215550419093[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_475369215550419093[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_475369215550419093[38] = 0;
   out_475369215550419093[39] = -state[7];
   out_475369215550419093[40] = state[6];
   out_475369215550419093[41] = 0;
   out_475369215550419093[42] = state[4];
   out_475369215550419093[43] = -state[3];
   out_475369215550419093[44] = 0;
   out_475369215550419093[45] = 0;
   out_475369215550419093[46] = 0;
   out_475369215550419093[47] = 0;
   out_475369215550419093[48] = 0;
   out_475369215550419093[49] = 0;
   out_475369215550419093[50] = 1;
   out_475369215550419093[51] = 0;
   out_475369215550419093[52] = 0;
   out_475369215550419093[53] = 1;
}
void h_13(double *state, double *unused, double *out_2054920899761121640) {
   out_2054920899761121640[0] = state[3];
   out_2054920899761121640[1] = state[4];
   out_2054920899761121640[2] = state[5];
}
void H_13(double *state, double *unused, double *out_8200288738589121772) {
   out_8200288738589121772[0] = 0;
   out_8200288738589121772[1] = 0;
   out_8200288738589121772[2] = 0;
   out_8200288738589121772[3] = 1;
   out_8200288738589121772[4] = 0;
   out_8200288738589121772[5] = 0;
   out_8200288738589121772[6] = 0;
   out_8200288738589121772[7] = 0;
   out_8200288738589121772[8] = 0;
   out_8200288738589121772[9] = 0;
   out_8200288738589121772[10] = 0;
   out_8200288738589121772[11] = 0;
   out_8200288738589121772[12] = 0;
   out_8200288738589121772[13] = 0;
   out_8200288738589121772[14] = 0;
   out_8200288738589121772[15] = 0;
   out_8200288738589121772[16] = 0;
   out_8200288738589121772[17] = 0;
   out_8200288738589121772[18] = 0;
   out_8200288738589121772[19] = 0;
   out_8200288738589121772[20] = 0;
   out_8200288738589121772[21] = 0;
   out_8200288738589121772[22] = 1;
   out_8200288738589121772[23] = 0;
   out_8200288738589121772[24] = 0;
   out_8200288738589121772[25] = 0;
   out_8200288738589121772[26] = 0;
   out_8200288738589121772[27] = 0;
   out_8200288738589121772[28] = 0;
   out_8200288738589121772[29] = 0;
   out_8200288738589121772[30] = 0;
   out_8200288738589121772[31] = 0;
   out_8200288738589121772[32] = 0;
   out_8200288738589121772[33] = 0;
   out_8200288738589121772[34] = 0;
   out_8200288738589121772[35] = 0;
   out_8200288738589121772[36] = 0;
   out_8200288738589121772[37] = 0;
   out_8200288738589121772[38] = 0;
   out_8200288738589121772[39] = 0;
   out_8200288738589121772[40] = 0;
   out_8200288738589121772[41] = 1;
   out_8200288738589121772[42] = 0;
   out_8200288738589121772[43] = 0;
   out_8200288738589121772[44] = 0;
   out_8200288738589121772[45] = 0;
   out_8200288738589121772[46] = 0;
   out_8200288738589121772[47] = 0;
   out_8200288738589121772[48] = 0;
   out_8200288738589121772[49] = 0;
   out_8200288738589121772[50] = 0;
   out_8200288738589121772[51] = 0;
   out_8200288738589121772[52] = 0;
   out_8200288738589121772[53] = 0;
}
void h_14(double *state, double *unused, double *out_3615713150161481771) {
   out_3615713150161481771[0] = state[6];
   out_3615713150161481771[1] = state[7];
   out_3615713150161481771[2] = state[8];
}
void H_14(double *state, double *unused, double *out_8951255769596273500) {
   out_8951255769596273500[0] = 0;
   out_8951255769596273500[1] = 0;
   out_8951255769596273500[2] = 0;
   out_8951255769596273500[3] = 0;
   out_8951255769596273500[4] = 0;
   out_8951255769596273500[5] = 0;
   out_8951255769596273500[6] = 1;
   out_8951255769596273500[7] = 0;
   out_8951255769596273500[8] = 0;
   out_8951255769596273500[9] = 0;
   out_8951255769596273500[10] = 0;
   out_8951255769596273500[11] = 0;
   out_8951255769596273500[12] = 0;
   out_8951255769596273500[13] = 0;
   out_8951255769596273500[14] = 0;
   out_8951255769596273500[15] = 0;
   out_8951255769596273500[16] = 0;
   out_8951255769596273500[17] = 0;
   out_8951255769596273500[18] = 0;
   out_8951255769596273500[19] = 0;
   out_8951255769596273500[20] = 0;
   out_8951255769596273500[21] = 0;
   out_8951255769596273500[22] = 0;
   out_8951255769596273500[23] = 0;
   out_8951255769596273500[24] = 0;
   out_8951255769596273500[25] = 1;
   out_8951255769596273500[26] = 0;
   out_8951255769596273500[27] = 0;
   out_8951255769596273500[28] = 0;
   out_8951255769596273500[29] = 0;
   out_8951255769596273500[30] = 0;
   out_8951255769596273500[31] = 0;
   out_8951255769596273500[32] = 0;
   out_8951255769596273500[33] = 0;
   out_8951255769596273500[34] = 0;
   out_8951255769596273500[35] = 0;
   out_8951255769596273500[36] = 0;
   out_8951255769596273500[37] = 0;
   out_8951255769596273500[38] = 0;
   out_8951255769596273500[39] = 0;
   out_8951255769596273500[40] = 0;
   out_8951255769596273500[41] = 0;
   out_8951255769596273500[42] = 0;
   out_8951255769596273500[43] = 0;
   out_8951255769596273500[44] = 1;
   out_8951255769596273500[45] = 0;
   out_8951255769596273500[46] = 0;
   out_8951255769596273500[47] = 0;
   out_8951255769596273500[48] = 0;
   out_8951255769596273500[49] = 0;
   out_8951255769596273500[50] = 0;
   out_8951255769596273500[51] = 0;
   out_8951255769596273500[52] = 0;
   out_8951255769596273500[53] = 0;
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

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_2903892006356229305) {
  err_fun(nom_x, delta_x, out_2903892006356229305);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_1008699991617682266) {
  inv_err_fun(nom_x, true_x, out_1008699991617682266);
}
void pose_H_mod_fun(double *state, double *out_8262236992047738224) {
  H_mod_fun(state, out_8262236992047738224);
}
void pose_f_fun(double *state, double dt, double *out_5777010685098588684) {
  f_fun(state,  dt, out_5777010685098588684);
}
void pose_F_fun(double *state, double dt, double *out_1785538860011231665) {
  F_fun(state,  dt, out_1785538860011231665);
}
void pose_h_4(double *state, double *unused, double *out_1907516981769351839) {
  h_4(state, unused, out_1907516981769351839);
}
void pose_H_4(double *state, double *unused, double *out_6412699871817905820) {
  H_4(state, unused, out_6412699871817905820);
}
void pose_h_10(double *state, double *unused, double *out_5032815439302672244) {
  h_10(state, unused, out_5032815439302672244);
}
void pose_H_10(double *state, double *unused, double *out_475369215550419093) {
  H_10(state, unused, out_475369215550419093);
}
void pose_h_13(double *state, double *unused, double *out_2054920899761121640) {
  h_13(state, unused, out_2054920899761121640);
}
void pose_H_13(double *state, double *unused, double *out_8200288738589121772) {
  H_13(state, unused, out_8200288738589121772);
}
void pose_h_14(double *state, double *unused, double *out_3615713150161481771) {
  h_14(state, unused, out_3615713150161481771);
}
void pose_H_14(double *state, double *unused, double *out_8951255769596273500) {
  H_14(state, unused, out_8951255769596273500);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
