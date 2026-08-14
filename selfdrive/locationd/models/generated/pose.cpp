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
void err_fun(double *nom_x, double *delta_x, double *out_7868974326132984287) {
   out_7868974326132984287[0] = delta_x[0] + nom_x[0];
   out_7868974326132984287[1] = delta_x[1] + nom_x[1];
   out_7868974326132984287[2] = delta_x[2] + nom_x[2];
   out_7868974326132984287[3] = delta_x[3] + nom_x[3];
   out_7868974326132984287[4] = delta_x[4] + nom_x[4];
   out_7868974326132984287[5] = delta_x[5] + nom_x[5];
   out_7868974326132984287[6] = delta_x[6] + nom_x[6];
   out_7868974326132984287[7] = delta_x[7] + nom_x[7];
   out_7868974326132984287[8] = delta_x[8] + nom_x[8];
   out_7868974326132984287[9] = delta_x[9] + nom_x[9];
   out_7868974326132984287[10] = delta_x[10] + nom_x[10];
   out_7868974326132984287[11] = delta_x[11] + nom_x[11];
   out_7868974326132984287[12] = delta_x[12] + nom_x[12];
   out_7868974326132984287[13] = delta_x[13] + nom_x[13];
   out_7868974326132984287[14] = delta_x[14] + nom_x[14];
   out_7868974326132984287[15] = delta_x[15] + nom_x[15];
   out_7868974326132984287[16] = delta_x[16] + nom_x[16];
   out_7868974326132984287[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1909625330139833135) {
   out_1909625330139833135[0] = -nom_x[0] + true_x[0];
   out_1909625330139833135[1] = -nom_x[1] + true_x[1];
   out_1909625330139833135[2] = -nom_x[2] + true_x[2];
   out_1909625330139833135[3] = -nom_x[3] + true_x[3];
   out_1909625330139833135[4] = -nom_x[4] + true_x[4];
   out_1909625330139833135[5] = -nom_x[5] + true_x[5];
   out_1909625330139833135[6] = -nom_x[6] + true_x[6];
   out_1909625330139833135[7] = -nom_x[7] + true_x[7];
   out_1909625330139833135[8] = -nom_x[8] + true_x[8];
   out_1909625330139833135[9] = -nom_x[9] + true_x[9];
   out_1909625330139833135[10] = -nom_x[10] + true_x[10];
   out_1909625330139833135[11] = -nom_x[11] + true_x[11];
   out_1909625330139833135[12] = -nom_x[12] + true_x[12];
   out_1909625330139833135[13] = -nom_x[13] + true_x[13];
   out_1909625330139833135[14] = -nom_x[14] + true_x[14];
   out_1909625330139833135[15] = -nom_x[15] + true_x[15];
   out_1909625330139833135[16] = -nom_x[16] + true_x[16];
   out_1909625330139833135[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_839159570712955856) {
   out_839159570712955856[0] = 1.0;
   out_839159570712955856[1] = 0.0;
   out_839159570712955856[2] = 0.0;
   out_839159570712955856[3] = 0.0;
   out_839159570712955856[4] = 0.0;
   out_839159570712955856[5] = 0.0;
   out_839159570712955856[6] = 0.0;
   out_839159570712955856[7] = 0.0;
   out_839159570712955856[8] = 0.0;
   out_839159570712955856[9] = 0.0;
   out_839159570712955856[10] = 0.0;
   out_839159570712955856[11] = 0.0;
   out_839159570712955856[12] = 0.0;
   out_839159570712955856[13] = 0.0;
   out_839159570712955856[14] = 0.0;
   out_839159570712955856[15] = 0.0;
   out_839159570712955856[16] = 0.0;
   out_839159570712955856[17] = 0.0;
   out_839159570712955856[18] = 0.0;
   out_839159570712955856[19] = 1.0;
   out_839159570712955856[20] = 0.0;
   out_839159570712955856[21] = 0.0;
   out_839159570712955856[22] = 0.0;
   out_839159570712955856[23] = 0.0;
   out_839159570712955856[24] = 0.0;
   out_839159570712955856[25] = 0.0;
   out_839159570712955856[26] = 0.0;
   out_839159570712955856[27] = 0.0;
   out_839159570712955856[28] = 0.0;
   out_839159570712955856[29] = 0.0;
   out_839159570712955856[30] = 0.0;
   out_839159570712955856[31] = 0.0;
   out_839159570712955856[32] = 0.0;
   out_839159570712955856[33] = 0.0;
   out_839159570712955856[34] = 0.0;
   out_839159570712955856[35] = 0.0;
   out_839159570712955856[36] = 0.0;
   out_839159570712955856[37] = 0.0;
   out_839159570712955856[38] = 1.0;
   out_839159570712955856[39] = 0.0;
   out_839159570712955856[40] = 0.0;
   out_839159570712955856[41] = 0.0;
   out_839159570712955856[42] = 0.0;
   out_839159570712955856[43] = 0.0;
   out_839159570712955856[44] = 0.0;
   out_839159570712955856[45] = 0.0;
   out_839159570712955856[46] = 0.0;
   out_839159570712955856[47] = 0.0;
   out_839159570712955856[48] = 0.0;
   out_839159570712955856[49] = 0.0;
   out_839159570712955856[50] = 0.0;
   out_839159570712955856[51] = 0.0;
   out_839159570712955856[52] = 0.0;
   out_839159570712955856[53] = 0.0;
   out_839159570712955856[54] = 0.0;
   out_839159570712955856[55] = 0.0;
   out_839159570712955856[56] = 0.0;
   out_839159570712955856[57] = 1.0;
   out_839159570712955856[58] = 0.0;
   out_839159570712955856[59] = 0.0;
   out_839159570712955856[60] = 0.0;
   out_839159570712955856[61] = 0.0;
   out_839159570712955856[62] = 0.0;
   out_839159570712955856[63] = 0.0;
   out_839159570712955856[64] = 0.0;
   out_839159570712955856[65] = 0.0;
   out_839159570712955856[66] = 0.0;
   out_839159570712955856[67] = 0.0;
   out_839159570712955856[68] = 0.0;
   out_839159570712955856[69] = 0.0;
   out_839159570712955856[70] = 0.0;
   out_839159570712955856[71] = 0.0;
   out_839159570712955856[72] = 0.0;
   out_839159570712955856[73] = 0.0;
   out_839159570712955856[74] = 0.0;
   out_839159570712955856[75] = 0.0;
   out_839159570712955856[76] = 1.0;
   out_839159570712955856[77] = 0.0;
   out_839159570712955856[78] = 0.0;
   out_839159570712955856[79] = 0.0;
   out_839159570712955856[80] = 0.0;
   out_839159570712955856[81] = 0.0;
   out_839159570712955856[82] = 0.0;
   out_839159570712955856[83] = 0.0;
   out_839159570712955856[84] = 0.0;
   out_839159570712955856[85] = 0.0;
   out_839159570712955856[86] = 0.0;
   out_839159570712955856[87] = 0.0;
   out_839159570712955856[88] = 0.0;
   out_839159570712955856[89] = 0.0;
   out_839159570712955856[90] = 0.0;
   out_839159570712955856[91] = 0.0;
   out_839159570712955856[92] = 0.0;
   out_839159570712955856[93] = 0.0;
   out_839159570712955856[94] = 0.0;
   out_839159570712955856[95] = 1.0;
   out_839159570712955856[96] = 0.0;
   out_839159570712955856[97] = 0.0;
   out_839159570712955856[98] = 0.0;
   out_839159570712955856[99] = 0.0;
   out_839159570712955856[100] = 0.0;
   out_839159570712955856[101] = 0.0;
   out_839159570712955856[102] = 0.0;
   out_839159570712955856[103] = 0.0;
   out_839159570712955856[104] = 0.0;
   out_839159570712955856[105] = 0.0;
   out_839159570712955856[106] = 0.0;
   out_839159570712955856[107] = 0.0;
   out_839159570712955856[108] = 0.0;
   out_839159570712955856[109] = 0.0;
   out_839159570712955856[110] = 0.0;
   out_839159570712955856[111] = 0.0;
   out_839159570712955856[112] = 0.0;
   out_839159570712955856[113] = 0.0;
   out_839159570712955856[114] = 1.0;
   out_839159570712955856[115] = 0.0;
   out_839159570712955856[116] = 0.0;
   out_839159570712955856[117] = 0.0;
   out_839159570712955856[118] = 0.0;
   out_839159570712955856[119] = 0.0;
   out_839159570712955856[120] = 0.0;
   out_839159570712955856[121] = 0.0;
   out_839159570712955856[122] = 0.0;
   out_839159570712955856[123] = 0.0;
   out_839159570712955856[124] = 0.0;
   out_839159570712955856[125] = 0.0;
   out_839159570712955856[126] = 0.0;
   out_839159570712955856[127] = 0.0;
   out_839159570712955856[128] = 0.0;
   out_839159570712955856[129] = 0.0;
   out_839159570712955856[130] = 0.0;
   out_839159570712955856[131] = 0.0;
   out_839159570712955856[132] = 0.0;
   out_839159570712955856[133] = 1.0;
   out_839159570712955856[134] = 0.0;
   out_839159570712955856[135] = 0.0;
   out_839159570712955856[136] = 0.0;
   out_839159570712955856[137] = 0.0;
   out_839159570712955856[138] = 0.0;
   out_839159570712955856[139] = 0.0;
   out_839159570712955856[140] = 0.0;
   out_839159570712955856[141] = 0.0;
   out_839159570712955856[142] = 0.0;
   out_839159570712955856[143] = 0.0;
   out_839159570712955856[144] = 0.0;
   out_839159570712955856[145] = 0.0;
   out_839159570712955856[146] = 0.0;
   out_839159570712955856[147] = 0.0;
   out_839159570712955856[148] = 0.0;
   out_839159570712955856[149] = 0.0;
   out_839159570712955856[150] = 0.0;
   out_839159570712955856[151] = 0.0;
   out_839159570712955856[152] = 1.0;
   out_839159570712955856[153] = 0.0;
   out_839159570712955856[154] = 0.0;
   out_839159570712955856[155] = 0.0;
   out_839159570712955856[156] = 0.0;
   out_839159570712955856[157] = 0.0;
   out_839159570712955856[158] = 0.0;
   out_839159570712955856[159] = 0.0;
   out_839159570712955856[160] = 0.0;
   out_839159570712955856[161] = 0.0;
   out_839159570712955856[162] = 0.0;
   out_839159570712955856[163] = 0.0;
   out_839159570712955856[164] = 0.0;
   out_839159570712955856[165] = 0.0;
   out_839159570712955856[166] = 0.0;
   out_839159570712955856[167] = 0.0;
   out_839159570712955856[168] = 0.0;
   out_839159570712955856[169] = 0.0;
   out_839159570712955856[170] = 0.0;
   out_839159570712955856[171] = 1.0;
   out_839159570712955856[172] = 0.0;
   out_839159570712955856[173] = 0.0;
   out_839159570712955856[174] = 0.0;
   out_839159570712955856[175] = 0.0;
   out_839159570712955856[176] = 0.0;
   out_839159570712955856[177] = 0.0;
   out_839159570712955856[178] = 0.0;
   out_839159570712955856[179] = 0.0;
   out_839159570712955856[180] = 0.0;
   out_839159570712955856[181] = 0.0;
   out_839159570712955856[182] = 0.0;
   out_839159570712955856[183] = 0.0;
   out_839159570712955856[184] = 0.0;
   out_839159570712955856[185] = 0.0;
   out_839159570712955856[186] = 0.0;
   out_839159570712955856[187] = 0.0;
   out_839159570712955856[188] = 0.0;
   out_839159570712955856[189] = 0.0;
   out_839159570712955856[190] = 1.0;
   out_839159570712955856[191] = 0.0;
   out_839159570712955856[192] = 0.0;
   out_839159570712955856[193] = 0.0;
   out_839159570712955856[194] = 0.0;
   out_839159570712955856[195] = 0.0;
   out_839159570712955856[196] = 0.0;
   out_839159570712955856[197] = 0.0;
   out_839159570712955856[198] = 0.0;
   out_839159570712955856[199] = 0.0;
   out_839159570712955856[200] = 0.0;
   out_839159570712955856[201] = 0.0;
   out_839159570712955856[202] = 0.0;
   out_839159570712955856[203] = 0.0;
   out_839159570712955856[204] = 0.0;
   out_839159570712955856[205] = 0.0;
   out_839159570712955856[206] = 0.0;
   out_839159570712955856[207] = 0.0;
   out_839159570712955856[208] = 0.0;
   out_839159570712955856[209] = 1.0;
   out_839159570712955856[210] = 0.0;
   out_839159570712955856[211] = 0.0;
   out_839159570712955856[212] = 0.0;
   out_839159570712955856[213] = 0.0;
   out_839159570712955856[214] = 0.0;
   out_839159570712955856[215] = 0.0;
   out_839159570712955856[216] = 0.0;
   out_839159570712955856[217] = 0.0;
   out_839159570712955856[218] = 0.0;
   out_839159570712955856[219] = 0.0;
   out_839159570712955856[220] = 0.0;
   out_839159570712955856[221] = 0.0;
   out_839159570712955856[222] = 0.0;
   out_839159570712955856[223] = 0.0;
   out_839159570712955856[224] = 0.0;
   out_839159570712955856[225] = 0.0;
   out_839159570712955856[226] = 0.0;
   out_839159570712955856[227] = 0.0;
   out_839159570712955856[228] = 1.0;
   out_839159570712955856[229] = 0.0;
   out_839159570712955856[230] = 0.0;
   out_839159570712955856[231] = 0.0;
   out_839159570712955856[232] = 0.0;
   out_839159570712955856[233] = 0.0;
   out_839159570712955856[234] = 0.0;
   out_839159570712955856[235] = 0.0;
   out_839159570712955856[236] = 0.0;
   out_839159570712955856[237] = 0.0;
   out_839159570712955856[238] = 0.0;
   out_839159570712955856[239] = 0.0;
   out_839159570712955856[240] = 0.0;
   out_839159570712955856[241] = 0.0;
   out_839159570712955856[242] = 0.0;
   out_839159570712955856[243] = 0.0;
   out_839159570712955856[244] = 0.0;
   out_839159570712955856[245] = 0.0;
   out_839159570712955856[246] = 0.0;
   out_839159570712955856[247] = 1.0;
   out_839159570712955856[248] = 0.0;
   out_839159570712955856[249] = 0.0;
   out_839159570712955856[250] = 0.0;
   out_839159570712955856[251] = 0.0;
   out_839159570712955856[252] = 0.0;
   out_839159570712955856[253] = 0.0;
   out_839159570712955856[254] = 0.0;
   out_839159570712955856[255] = 0.0;
   out_839159570712955856[256] = 0.0;
   out_839159570712955856[257] = 0.0;
   out_839159570712955856[258] = 0.0;
   out_839159570712955856[259] = 0.0;
   out_839159570712955856[260] = 0.0;
   out_839159570712955856[261] = 0.0;
   out_839159570712955856[262] = 0.0;
   out_839159570712955856[263] = 0.0;
   out_839159570712955856[264] = 0.0;
   out_839159570712955856[265] = 0.0;
   out_839159570712955856[266] = 1.0;
   out_839159570712955856[267] = 0.0;
   out_839159570712955856[268] = 0.0;
   out_839159570712955856[269] = 0.0;
   out_839159570712955856[270] = 0.0;
   out_839159570712955856[271] = 0.0;
   out_839159570712955856[272] = 0.0;
   out_839159570712955856[273] = 0.0;
   out_839159570712955856[274] = 0.0;
   out_839159570712955856[275] = 0.0;
   out_839159570712955856[276] = 0.0;
   out_839159570712955856[277] = 0.0;
   out_839159570712955856[278] = 0.0;
   out_839159570712955856[279] = 0.0;
   out_839159570712955856[280] = 0.0;
   out_839159570712955856[281] = 0.0;
   out_839159570712955856[282] = 0.0;
   out_839159570712955856[283] = 0.0;
   out_839159570712955856[284] = 0.0;
   out_839159570712955856[285] = 1.0;
   out_839159570712955856[286] = 0.0;
   out_839159570712955856[287] = 0.0;
   out_839159570712955856[288] = 0.0;
   out_839159570712955856[289] = 0.0;
   out_839159570712955856[290] = 0.0;
   out_839159570712955856[291] = 0.0;
   out_839159570712955856[292] = 0.0;
   out_839159570712955856[293] = 0.0;
   out_839159570712955856[294] = 0.0;
   out_839159570712955856[295] = 0.0;
   out_839159570712955856[296] = 0.0;
   out_839159570712955856[297] = 0.0;
   out_839159570712955856[298] = 0.0;
   out_839159570712955856[299] = 0.0;
   out_839159570712955856[300] = 0.0;
   out_839159570712955856[301] = 0.0;
   out_839159570712955856[302] = 0.0;
   out_839159570712955856[303] = 0.0;
   out_839159570712955856[304] = 1.0;
   out_839159570712955856[305] = 0.0;
   out_839159570712955856[306] = 0.0;
   out_839159570712955856[307] = 0.0;
   out_839159570712955856[308] = 0.0;
   out_839159570712955856[309] = 0.0;
   out_839159570712955856[310] = 0.0;
   out_839159570712955856[311] = 0.0;
   out_839159570712955856[312] = 0.0;
   out_839159570712955856[313] = 0.0;
   out_839159570712955856[314] = 0.0;
   out_839159570712955856[315] = 0.0;
   out_839159570712955856[316] = 0.0;
   out_839159570712955856[317] = 0.0;
   out_839159570712955856[318] = 0.0;
   out_839159570712955856[319] = 0.0;
   out_839159570712955856[320] = 0.0;
   out_839159570712955856[321] = 0.0;
   out_839159570712955856[322] = 0.0;
   out_839159570712955856[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_6870720994996901920) {
   out_6870720994996901920[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_6870720994996901920[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_6870720994996901920[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_6870720994996901920[3] = dt*state[12] + state[3];
   out_6870720994996901920[4] = dt*state[13] + state[4];
   out_6870720994996901920[5] = dt*state[14] + state[5];
   out_6870720994996901920[6] = state[6];
   out_6870720994996901920[7] = state[7];
   out_6870720994996901920[8] = state[8];
   out_6870720994996901920[9] = state[9];
   out_6870720994996901920[10] = state[10];
   out_6870720994996901920[11] = state[11];
   out_6870720994996901920[12] = state[12];
   out_6870720994996901920[13] = state[13];
   out_6870720994996901920[14] = state[14];
   out_6870720994996901920[15] = state[15];
   out_6870720994996901920[16] = state[16];
   out_6870720994996901920[17] = state[17];
}
void F_fun(double *state, double dt, double *out_8965886230000449482) {
   out_8965886230000449482[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8965886230000449482[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8965886230000449482[2] = 0;
   out_8965886230000449482[3] = 0;
   out_8965886230000449482[4] = 0;
   out_8965886230000449482[5] = 0;
   out_8965886230000449482[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8965886230000449482[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8965886230000449482[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8965886230000449482[9] = 0;
   out_8965886230000449482[10] = 0;
   out_8965886230000449482[11] = 0;
   out_8965886230000449482[12] = 0;
   out_8965886230000449482[13] = 0;
   out_8965886230000449482[14] = 0;
   out_8965886230000449482[15] = 0;
   out_8965886230000449482[16] = 0;
   out_8965886230000449482[17] = 0;
   out_8965886230000449482[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8965886230000449482[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8965886230000449482[20] = 0;
   out_8965886230000449482[21] = 0;
   out_8965886230000449482[22] = 0;
   out_8965886230000449482[23] = 0;
   out_8965886230000449482[24] = 0;
   out_8965886230000449482[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8965886230000449482[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8965886230000449482[27] = 0;
   out_8965886230000449482[28] = 0;
   out_8965886230000449482[29] = 0;
   out_8965886230000449482[30] = 0;
   out_8965886230000449482[31] = 0;
   out_8965886230000449482[32] = 0;
   out_8965886230000449482[33] = 0;
   out_8965886230000449482[34] = 0;
   out_8965886230000449482[35] = 0;
   out_8965886230000449482[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8965886230000449482[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8965886230000449482[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8965886230000449482[39] = 0;
   out_8965886230000449482[40] = 0;
   out_8965886230000449482[41] = 0;
   out_8965886230000449482[42] = 0;
   out_8965886230000449482[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8965886230000449482[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8965886230000449482[45] = 0;
   out_8965886230000449482[46] = 0;
   out_8965886230000449482[47] = 0;
   out_8965886230000449482[48] = 0;
   out_8965886230000449482[49] = 0;
   out_8965886230000449482[50] = 0;
   out_8965886230000449482[51] = 0;
   out_8965886230000449482[52] = 0;
   out_8965886230000449482[53] = 0;
   out_8965886230000449482[54] = 0;
   out_8965886230000449482[55] = 0;
   out_8965886230000449482[56] = 0;
   out_8965886230000449482[57] = 1;
   out_8965886230000449482[58] = 0;
   out_8965886230000449482[59] = 0;
   out_8965886230000449482[60] = 0;
   out_8965886230000449482[61] = 0;
   out_8965886230000449482[62] = 0;
   out_8965886230000449482[63] = 0;
   out_8965886230000449482[64] = 0;
   out_8965886230000449482[65] = 0;
   out_8965886230000449482[66] = dt;
   out_8965886230000449482[67] = 0;
   out_8965886230000449482[68] = 0;
   out_8965886230000449482[69] = 0;
   out_8965886230000449482[70] = 0;
   out_8965886230000449482[71] = 0;
   out_8965886230000449482[72] = 0;
   out_8965886230000449482[73] = 0;
   out_8965886230000449482[74] = 0;
   out_8965886230000449482[75] = 0;
   out_8965886230000449482[76] = 1;
   out_8965886230000449482[77] = 0;
   out_8965886230000449482[78] = 0;
   out_8965886230000449482[79] = 0;
   out_8965886230000449482[80] = 0;
   out_8965886230000449482[81] = 0;
   out_8965886230000449482[82] = 0;
   out_8965886230000449482[83] = 0;
   out_8965886230000449482[84] = 0;
   out_8965886230000449482[85] = dt;
   out_8965886230000449482[86] = 0;
   out_8965886230000449482[87] = 0;
   out_8965886230000449482[88] = 0;
   out_8965886230000449482[89] = 0;
   out_8965886230000449482[90] = 0;
   out_8965886230000449482[91] = 0;
   out_8965886230000449482[92] = 0;
   out_8965886230000449482[93] = 0;
   out_8965886230000449482[94] = 0;
   out_8965886230000449482[95] = 1;
   out_8965886230000449482[96] = 0;
   out_8965886230000449482[97] = 0;
   out_8965886230000449482[98] = 0;
   out_8965886230000449482[99] = 0;
   out_8965886230000449482[100] = 0;
   out_8965886230000449482[101] = 0;
   out_8965886230000449482[102] = 0;
   out_8965886230000449482[103] = 0;
   out_8965886230000449482[104] = dt;
   out_8965886230000449482[105] = 0;
   out_8965886230000449482[106] = 0;
   out_8965886230000449482[107] = 0;
   out_8965886230000449482[108] = 0;
   out_8965886230000449482[109] = 0;
   out_8965886230000449482[110] = 0;
   out_8965886230000449482[111] = 0;
   out_8965886230000449482[112] = 0;
   out_8965886230000449482[113] = 0;
   out_8965886230000449482[114] = 1;
   out_8965886230000449482[115] = 0;
   out_8965886230000449482[116] = 0;
   out_8965886230000449482[117] = 0;
   out_8965886230000449482[118] = 0;
   out_8965886230000449482[119] = 0;
   out_8965886230000449482[120] = 0;
   out_8965886230000449482[121] = 0;
   out_8965886230000449482[122] = 0;
   out_8965886230000449482[123] = 0;
   out_8965886230000449482[124] = 0;
   out_8965886230000449482[125] = 0;
   out_8965886230000449482[126] = 0;
   out_8965886230000449482[127] = 0;
   out_8965886230000449482[128] = 0;
   out_8965886230000449482[129] = 0;
   out_8965886230000449482[130] = 0;
   out_8965886230000449482[131] = 0;
   out_8965886230000449482[132] = 0;
   out_8965886230000449482[133] = 1;
   out_8965886230000449482[134] = 0;
   out_8965886230000449482[135] = 0;
   out_8965886230000449482[136] = 0;
   out_8965886230000449482[137] = 0;
   out_8965886230000449482[138] = 0;
   out_8965886230000449482[139] = 0;
   out_8965886230000449482[140] = 0;
   out_8965886230000449482[141] = 0;
   out_8965886230000449482[142] = 0;
   out_8965886230000449482[143] = 0;
   out_8965886230000449482[144] = 0;
   out_8965886230000449482[145] = 0;
   out_8965886230000449482[146] = 0;
   out_8965886230000449482[147] = 0;
   out_8965886230000449482[148] = 0;
   out_8965886230000449482[149] = 0;
   out_8965886230000449482[150] = 0;
   out_8965886230000449482[151] = 0;
   out_8965886230000449482[152] = 1;
   out_8965886230000449482[153] = 0;
   out_8965886230000449482[154] = 0;
   out_8965886230000449482[155] = 0;
   out_8965886230000449482[156] = 0;
   out_8965886230000449482[157] = 0;
   out_8965886230000449482[158] = 0;
   out_8965886230000449482[159] = 0;
   out_8965886230000449482[160] = 0;
   out_8965886230000449482[161] = 0;
   out_8965886230000449482[162] = 0;
   out_8965886230000449482[163] = 0;
   out_8965886230000449482[164] = 0;
   out_8965886230000449482[165] = 0;
   out_8965886230000449482[166] = 0;
   out_8965886230000449482[167] = 0;
   out_8965886230000449482[168] = 0;
   out_8965886230000449482[169] = 0;
   out_8965886230000449482[170] = 0;
   out_8965886230000449482[171] = 1;
   out_8965886230000449482[172] = 0;
   out_8965886230000449482[173] = 0;
   out_8965886230000449482[174] = 0;
   out_8965886230000449482[175] = 0;
   out_8965886230000449482[176] = 0;
   out_8965886230000449482[177] = 0;
   out_8965886230000449482[178] = 0;
   out_8965886230000449482[179] = 0;
   out_8965886230000449482[180] = 0;
   out_8965886230000449482[181] = 0;
   out_8965886230000449482[182] = 0;
   out_8965886230000449482[183] = 0;
   out_8965886230000449482[184] = 0;
   out_8965886230000449482[185] = 0;
   out_8965886230000449482[186] = 0;
   out_8965886230000449482[187] = 0;
   out_8965886230000449482[188] = 0;
   out_8965886230000449482[189] = 0;
   out_8965886230000449482[190] = 1;
   out_8965886230000449482[191] = 0;
   out_8965886230000449482[192] = 0;
   out_8965886230000449482[193] = 0;
   out_8965886230000449482[194] = 0;
   out_8965886230000449482[195] = 0;
   out_8965886230000449482[196] = 0;
   out_8965886230000449482[197] = 0;
   out_8965886230000449482[198] = 0;
   out_8965886230000449482[199] = 0;
   out_8965886230000449482[200] = 0;
   out_8965886230000449482[201] = 0;
   out_8965886230000449482[202] = 0;
   out_8965886230000449482[203] = 0;
   out_8965886230000449482[204] = 0;
   out_8965886230000449482[205] = 0;
   out_8965886230000449482[206] = 0;
   out_8965886230000449482[207] = 0;
   out_8965886230000449482[208] = 0;
   out_8965886230000449482[209] = 1;
   out_8965886230000449482[210] = 0;
   out_8965886230000449482[211] = 0;
   out_8965886230000449482[212] = 0;
   out_8965886230000449482[213] = 0;
   out_8965886230000449482[214] = 0;
   out_8965886230000449482[215] = 0;
   out_8965886230000449482[216] = 0;
   out_8965886230000449482[217] = 0;
   out_8965886230000449482[218] = 0;
   out_8965886230000449482[219] = 0;
   out_8965886230000449482[220] = 0;
   out_8965886230000449482[221] = 0;
   out_8965886230000449482[222] = 0;
   out_8965886230000449482[223] = 0;
   out_8965886230000449482[224] = 0;
   out_8965886230000449482[225] = 0;
   out_8965886230000449482[226] = 0;
   out_8965886230000449482[227] = 0;
   out_8965886230000449482[228] = 1;
   out_8965886230000449482[229] = 0;
   out_8965886230000449482[230] = 0;
   out_8965886230000449482[231] = 0;
   out_8965886230000449482[232] = 0;
   out_8965886230000449482[233] = 0;
   out_8965886230000449482[234] = 0;
   out_8965886230000449482[235] = 0;
   out_8965886230000449482[236] = 0;
   out_8965886230000449482[237] = 0;
   out_8965886230000449482[238] = 0;
   out_8965886230000449482[239] = 0;
   out_8965886230000449482[240] = 0;
   out_8965886230000449482[241] = 0;
   out_8965886230000449482[242] = 0;
   out_8965886230000449482[243] = 0;
   out_8965886230000449482[244] = 0;
   out_8965886230000449482[245] = 0;
   out_8965886230000449482[246] = 0;
   out_8965886230000449482[247] = 1;
   out_8965886230000449482[248] = 0;
   out_8965886230000449482[249] = 0;
   out_8965886230000449482[250] = 0;
   out_8965886230000449482[251] = 0;
   out_8965886230000449482[252] = 0;
   out_8965886230000449482[253] = 0;
   out_8965886230000449482[254] = 0;
   out_8965886230000449482[255] = 0;
   out_8965886230000449482[256] = 0;
   out_8965886230000449482[257] = 0;
   out_8965886230000449482[258] = 0;
   out_8965886230000449482[259] = 0;
   out_8965886230000449482[260] = 0;
   out_8965886230000449482[261] = 0;
   out_8965886230000449482[262] = 0;
   out_8965886230000449482[263] = 0;
   out_8965886230000449482[264] = 0;
   out_8965886230000449482[265] = 0;
   out_8965886230000449482[266] = 1;
   out_8965886230000449482[267] = 0;
   out_8965886230000449482[268] = 0;
   out_8965886230000449482[269] = 0;
   out_8965886230000449482[270] = 0;
   out_8965886230000449482[271] = 0;
   out_8965886230000449482[272] = 0;
   out_8965886230000449482[273] = 0;
   out_8965886230000449482[274] = 0;
   out_8965886230000449482[275] = 0;
   out_8965886230000449482[276] = 0;
   out_8965886230000449482[277] = 0;
   out_8965886230000449482[278] = 0;
   out_8965886230000449482[279] = 0;
   out_8965886230000449482[280] = 0;
   out_8965886230000449482[281] = 0;
   out_8965886230000449482[282] = 0;
   out_8965886230000449482[283] = 0;
   out_8965886230000449482[284] = 0;
   out_8965886230000449482[285] = 1;
   out_8965886230000449482[286] = 0;
   out_8965886230000449482[287] = 0;
   out_8965886230000449482[288] = 0;
   out_8965886230000449482[289] = 0;
   out_8965886230000449482[290] = 0;
   out_8965886230000449482[291] = 0;
   out_8965886230000449482[292] = 0;
   out_8965886230000449482[293] = 0;
   out_8965886230000449482[294] = 0;
   out_8965886230000449482[295] = 0;
   out_8965886230000449482[296] = 0;
   out_8965886230000449482[297] = 0;
   out_8965886230000449482[298] = 0;
   out_8965886230000449482[299] = 0;
   out_8965886230000449482[300] = 0;
   out_8965886230000449482[301] = 0;
   out_8965886230000449482[302] = 0;
   out_8965886230000449482[303] = 0;
   out_8965886230000449482[304] = 1;
   out_8965886230000449482[305] = 0;
   out_8965886230000449482[306] = 0;
   out_8965886230000449482[307] = 0;
   out_8965886230000449482[308] = 0;
   out_8965886230000449482[309] = 0;
   out_8965886230000449482[310] = 0;
   out_8965886230000449482[311] = 0;
   out_8965886230000449482[312] = 0;
   out_8965886230000449482[313] = 0;
   out_8965886230000449482[314] = 0;
   out_8965886230000449482[315] = 0;
   out_8965886230000449482[316] = 0;
   out_8965886230000449482[317] = 0;
   out_8965886230000449482[318] = 0;
   out_8965886230000449482[319] = 0;
   out_8965886230000449482[320] = 0;
   out_8965886230000449482[321] = 0;
   out_8965886230000449482[322] = 0;
   out_8965886230000449482[323] = 1;
}
void h_4(double *state, double *unused, double *out_997896127340448425) {
   out_997896127340448425[0] = state[6] + state[9];
   out_997896127340448425[1] = state[7] + state[10];
   out_997896127340448425[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_4113381649503905109) {
   out_4113381649503905109[0] = 0;
   out_4113381649503905109[1] = 0;
   out_4113381649503905109[2] = 0;
   out_4113381649503905109[3] = 0;
   out_4113381649503905109[4] = 0;
   out_4113381649503905109[5] = 0;
   out_4113381649503905109[6] = 1;
   out_4113381649503905109[7] = 0;
   out_4113381649503905109[8] = 0;
   out_4113381649503905109[9] = 1;
   out_4113381649503905109[10] = 0;
   out_4113381649503905109[11] = 0;
   out_4113381649503905109[12] = 0;
   out_4113381649503905109[13] = 0;
   out_4113381649503905109[14] = 0;
   out_4113381649503905109[15] = 0;
   out_4113381649503905109[16] = 0;
   out_4113381649503905109[17] = 0;
   out_4113381649503905109[18] = 0;
   out_4113381649503905109[19] = 0;
   out_4113381649503905109[20] = 0;
   out_4113381649503905109[21] = 0;
   out_4113381649503905109[22] = 0;
   out_4113381649503905109[23] = 0;
   out_4113381649503905109[24] = 0;
   out_4113381649503905109[25] = 1;
   out_4113381649503905109[26] = 0;
   out_4113381649503905109[27] = 0;
   out_4113381649503905109[28] = 1;
   out_4113381649503905109[29] = 0;
   out_4113381649503905109[30] = 0;
   out_4113381649503905109[31] = 0;
   out_4113381649503905109[32] = 0;
   out_4113381649503905109[33] = 0;
   out_4113381649503905109[34] = 0;
   out_4113381649503905109[35] = 0;
   out_4113381649503905109[36] = 0;
   out_4113381649503905109[37] = 0;
   out_4113381649503905109[38] = 0;
   out_4113381649503905109[39] = 0;
   out_4113381649503905109[40] = 0;
   out_4113381649503905109[41] = 0;
   out_4113381649503905109[42] = 0;
   out_4113381649503905109[43] = 0;
   out_4113381649503905109[44] = 1;
   out_4113381649503905109[45] = 0;
   out_4113381649503905109[46] = 0;
   out_4113381649503905109[47] = 1;
   out_4113381649503905109[48] = 0;
   out_4113381649503905109[49] = 0;
   out_4113381649503905109[50] = 0;
   out_4113381649503905109[51] = 0;
   out_4113381649503905109[52] = 0;
   out_4113381649503905109[53] = 0;
}
void h_10(double *state, double *unused, double *out_8849325349390195320) {
   out_8849325349390195320[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_8849325349390195320[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_8849325349390195320[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_3026541354882437243) {
   out_3026541354882437243[0] = 0;
   out_3026541354882437243[1] = 9.8100000000000005*cos(state[1]);
   out_3026541354882437243[2] = 0;
   out_3026541354882437243[3] = 0;
   out_3026541354882437243[4] = -state[8];
   out_3026541354882437243[5] = state[7];
   out_3026541354882437243[6] = 0;
   out_3026541354882437243[7] = state[5];
   out_3026541354882437243[8] = -state[4];
   out_3026541354882437243[9] = 0;
   out_3026541354882437243[10] = 0;
   out_3026541354882437243[11] = 0;
   out_3026541354882437243[12] = 1;
   out_3026541354882437243[13] = 0;
   out_3026541354882437243[14] = 0;
   out_3026541354882437243[15] = 1;
   out_3026541354882437243[16] = 0;
   out_3026541354882437243[17] = 0;
   out_3026541354882437243[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_3026541354882437243[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_3026541354882437243[20] = 0;
   out_3026541354882437243[21] = state[8];
   out_3026541354882437243[22] = 0;
   out_3026541354882437243[23] = -state[6];
   out_3026541354882437243[24] = -state[5];
   out_3026541354882437243[25] = 0;
   out_3026541354882437243[26] = state[3];
   out_3026541354882437243[27] = 0;
   out_3026541354882437243[28] = 0;
   out_3026541354882437243[29] = 0;
   out_3026541354882437243[30] = 0;
   out_3026541354882437243[31] = 1;
   out_3026541354882437243[32] = 0;
   out_3026541354882437243[33] = 0;
   out_3026541354882437243[34] = 1;
   out_3026541354882437243[35] = 0;
   out_3026541354882437243[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_3026541354882437243[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_3026541354882437243[38] = 0;
   out_3026541354882437243[39] = -state[7];
   out_3026541354882437243[40] = state[6];
   out_3026541354882437243[41] = 0;
   out_3026541354882437243[42] = state[4];
   out_3026541354882437243[43] = -state[3];
   out_3026541354882437243[44] = 0;
   out_3026541354882437243[45] = 0;
   out_3026541354882437243[46] = 0;
   out_3026541354882437243[47] = 0;
   out_3026541354882437243[48] = 0;
   out_3026541354882437243[49] = 0;
   out_3026541354882437243[50] = 1;
   out_3026541354882437243[51] = 0;
   out_3026541354882437243[52] = 0;
   out_3026541354882437243[53] = 1;
}
void h_13(double *state, double *unused, double *out_8579573593214207751) {
   out_8579573593214207751[0] = state[3];
   out_8579573593214207751[1] = state[4];
   out_8579573593214207751[2] = state[5];
}
void H_13(double *state, double *unused, double *out_901107824171572308) {
   out_901107824171572308[0] = 0;
   out_901107824171572308[1] = 0;
   out_901107824171572308[2] = 0;
   out_901107824171572308[3] = 1;
   out_901107824171572308[4] = 0;
   out_901107824171572308[5] = 0;
   out_901107824171572308[6] = 0;
   out_901107824171572308[7] = 0;
   out_901107824171572308[8] = 0;
   out_901107824171572308[9] = 0;
   out_901107824171572308[10] = 0;
   out_901107824171572308[11] = 0;
   out_901107824171572308[12] = 0;
   out_901107824171572308[13] = 0;
   out_901107824171572308[14] = 0;
   out_901107824171572308[15] = 0;
   out_901107824171572308[16] = 0;
   out_901107824171572308[17] = 0;
   out_901107824171572308[18] = 0;
   out_901107824171572308[19] = 0;
   out_901107824171572308[20] = 0;
   out_901107824171572308[21] = 0;
   out_901107824171572308[22] = 1;
   out_901107824171572308[23] = 0;
   out_901107824171572308[24] = 0;
   out_901107824171572308[25] = 0;
   out_901107824171572308[26] = 0;
   out_901107824171572308[27] = 0;
   out_901107824171572308[28] = 0;
   out_901107824171572308[29] = 0;
   out_901107824171572308[30] = 0;
   out_901107824171572308[31] = 0;
   out_901107824171572308[32] = 0;
   out_901107824171572308[33] = 0;
   out_901107824171572308[34] = 0;
   out_901107824171572308[35] = 0;
   out_901107824171572308[36] = 0;
   out_901107824171572308[37] = 0;
   out_901107824171572308[38] = 0;
   out_901107824171572308[39] = 0;
   out_901107824171572308[40] = 0;
   out_901107824171572308[41] = 1;
   out_901107824171572308[42] = 0;
   out_901107824171572308[43] = 0;
   out_901107824171572308[44] = 0;
   out_901107824171572308[45] = 0;
   out_901107824171572308[46] = 0;
   out_901107824171572308[47] = 0;
   out_901107824171572308[48] = 0;
   out_901107824171572308[49] = 0;
   out_901107824171572308[50] = 0;
   out_901107824171572308[51] = 0;
   out_901107824171572308[52] = 0;
   out_901107824171572308[53] = 0;
}
void h_14(double *state, double *unused, double *out_4065584171885884559) {
   out_4065584171885884559[0] = state[6];
   out_4065584171885884559[1] = state[7];
   out_4065584171885884559[2] = state[8];
}
void H_14(double *state, double *unused, double *out_150140793164420580) {
   out_150140793164420580[0] = 0;
   out_150140793164420580[1] = 0;
   out_150140793164420580[2] = 0;
   out_150140793164420580[3] = 0;
   out_150140793164420580[4] = 0;
   out_150140793164420580[5] = 0;
   out_150140793164420580[6] = 1;
   out_150140793164420580[7] = 0;
   out_150140793164420580[8] = 0;
   out_150140793164420580[9] = 0;
   out_150140793164420580[10] = 0;
   out_150140793164420580[11] = 0;
   out_150140793164420580[12] = 0;
   out_150140793164420580[13] = 0;
   out_150140793164420580[14] = 0;
   out_150140793164420580[15] = 0;
   out_150140793164420580[16] = 0;
   out_150140793164420580[17] = 0;
   out_150140793164420580[18] = 0;
   out_150140793164420580[19] = 0;
   out_150140793164420580[20] = 0;
   out_150140793164420580[21] = 0;
   out_150140793164420580[22] = 0;
   out_150140793164420580[23] = 0;
   out_150140793164420580[24] = 0;
   out_150140793164420580[25] = 1;
   out_150140793164420580[26] = 0;
   out_150140793164420580[27] = 0;
   out_150140793164420580[28] = 0;
   out_150140793164420580[29] = 0;
   out_150140793164420580[30] = 0;
   out_150140793164420580[31] = 0;
   out_150140793164420580[32] = 0;
   out_150140793164420580[33] = 0;
   out_150140793164420580[34] = 0;
   out_150140793164420580[35] = 0;
   out_150140793164420580[36] = 0;
   out_150140793164420580[37] = 0;
   out_150140793164420580[38] = 0;
   out_150140793164420580[39] = 0;
   out_150140793164420580[40] = 0;
   out_150140793164420580[41] = 0;
   out_150140793164420580[42] = 0;
   out_150140793164420580[43] = 0;
   out_150140793164420580[44] = 1;
   out_150140793164420580[45] = 0;
   out_150140793164420580[46] = 0;
   out_150140793164420580[47] = 0;
   out_150140793164420580[48] = 0;
   out_150140793164420580[49] = 0;
   out_150140793164420580[50] = 0;
   out_150140793164420580[51] = 0;
   out_150140793164420580[52] = 0;
   out_150140793164420580[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_7868974326132984287) {
  err_fun(nom_x, delta_x, out_7868974326132984287);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_1909625330139833135) {
  inv_err_fun(nom_x, true_x, out_1909625330139833135);
}
void pose_H_mod_fun(double *state, double *out_839159570712955856) {
  H_mod_fun(state, out_839159570712955856);
}
void pose_f_fun(double *state, double dt, double *out_6870720994996901920) {
  f_fun(state,  dt, out_6870720994996901920);
}
void pose_F_fun(double *state, double dt, double *out_8965886230000449482) {
  F_fun(state,  dt, out_8965886230000449482);
}
void pose_h_4(double *state, double *unused, double *out_997896127340448425) {
  h_4(state, unused, out_997896127340448425);
}
void pose_H_4(double *state, double *unused, double *out_4113381649503905109) {
  H_4(state, unused, out_4113381649503905109);
}
void pose_h_10(double *state, double *unused, double *out_8849325349390195320) {
  h_10(state, unused, out_8849325349390195320);
}
void pose_H_10(double *state, double *unused, double *out_3026541354882437243) {
  H_10(state, unused, out_3026541354882437243);
}
void pose_h_13(double *state, double *unused, double *out_8579573593214207751) {
  h_13(state, unused, out_8579573593214207751);
}
void pose_H_13(double *state, double *unused, double *out_901107824171572308) {
  H_13(state, unused, out_901107824171572308);
}
void pose_h_14(double *state, double *unused, double *out_4065584171885884559) {
  h_14(state, unused, out_4065584171885884559);
}
void pose_H_14(double *state, double *unused, double *out_150140793164420580) {
  H_14(state, unused, out_150140793164420580);
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
