#ifndef SOS_COFF_H_
#define SOS_COFF_H_

#define SOS_COFF_NUM_OF_STAGES (2)

static filter_iir_2_f32_t s0 = FILTER_IIR_2_INIT(
1.86824893E-08f,
3.73649787E-08f,
1.86824893E-08f,
1.95685128E+00f,
-9.57394597E-01f);

static filter_iir_2_f32_t s1 = FILTER_IIR_2_INIT(
1.00000000E+00f,
2.00000000E+00f,
1.00000000E+00f,
1.98157908E+00f,
-9.82129258E-01f);

static filter_iir_2_f32_t * s[SOS_COFF_NUM_OF_STAGES] = {&s0, &s1};

#endif /* end of SOS_COFF_H_ */

