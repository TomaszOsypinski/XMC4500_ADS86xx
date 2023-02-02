# *******************************************************************************
## @file    sos_filter.py
## @version 1.0.0
## @date    2023-02-02
## @brief   Script to design low pass filter with scipy.signal
## @author  Tomasz Osypinski<br>

# Change History
# --------------

# 2023-02-02:
# - Initial
# *******************************************************************************

# Copyright (C) 2023, Tomasz Osypinski. All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:

# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.

import numpy as np
import scipy.signal as sig
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# Input parameters
fs               = 40000
f_pass           = 150
filter_order     = 4
filter_type      = 'butter'
filter_band_type = 'lowpass'
f_nyquist        = fs / 2

# Gain in measurement channel from ADC in to DAC output Vout_pp/Uin_pp
gain             = 2.2/(2* 1.5 * 4.096)

#'bandpass', 'lowpass', 'highpass’, 'bandstop'
#Butterworth :   'butter'
#Chebyshev I :   'cheby1'
#Chebyshev II :  'cheby2'
#Cauer/elliptic: 'ellip'

# Plot limits
freq_plot_min   = 0
freq_plot_max   = 1000
gain_plot_min   = -90
gain_plot_max   = 0
gain_plot_step  = 10
phase_plot_min  = -180
phase_plot_max  = 180
phase_plot_step = 45

# design filter
dp_iir = sig.iirfilter(filter_order, f_pass/f_nyquist, ftype=filter_type, btype=filter_band_type, output='sos')
shape = dp_iir.shape[0]

# frequency response
w, h_iir = sig.sosfreqz(dp_iir)
f = w * fs / (2 * np.pi)
gain_db = 20 * np.log10(np.abs(h_iir) * gain) 
phase_degree = np.degrees(np.angle(h_iir))

# coefficients numerator and denominator
coff_b, coff_a = sig.sos2tf(dp_iir)
np.warnings.filterwarnings('ignore')
group_delay = sig.group_delay((coff_b, coff_a))[1]
np.warnings.filterwarnings("default")

# impulse response
n_sumple_impuls = 150
impuls = sig.unit_impulse(n_sumple_impuls)
impuls_sos = sig.sosfilt(dp_iir, impuls)

# write coefficients
print('IIR SOS: y[n] = B0 * x[n] + B1 * x[n-1] + B2 * x[n-2] - A1 * y[n-1] - A2 * y[n-2]')
print('Parameters for stages [B0, B1, B2, 1, A1, A2]:\r\n', dp_iir)
print('Attention A1 and A2 from SOS output is multiplied by -1 when are written to file !')
mul_coff_A = -1

with open("sos_coff.h", 'w') as f_coff:
    f_coff.write('#ifndef SOS_COFF_H_\n')
    f_coff.write('#define SOS_COFF_H_\n\n')
    f_coff.write('#define SOS_COFF_NUM_OF_STAGES (' + str(shape) + ')\n\n')
    
    for i in range(0, shape):
        f_coff.write('static filter_iir_2_f32_t s' + str(i) + ' = FILTER_IIR_2_INIT(\n')
        f_coff.write('{:.8E}'.format(dp_iir[i][0]) + 'f,\n')            #B0
        f_coff.write('{:.8E}'.format(dp_iir[i][1]) + 'f,\n')            #B1
        f_coff.write('{:.8E}'.format(dp_iir[i][2]) + 'f,\n')            #B2
        f_coff.write('{:.8E}'.format(mul_coff_A * dp_iir[i][4]) + 'f,\n')       #A1
        f_coff.write('{:.8E}'.format(mul_coff_A * dp_iir[i][5]) + 'f);\n\n')    #A2

    f_coff.write('static filter_iir_2_f32_t * s[SOS_COFF_NUM_OF_STAGES] = {')
    for i in range(0, shape):
        f_coff.write('&s'+str(i))
        if i < (shape - 1):
            f_coff.write(', ')
    f_coff.write('};')
    f_coff.write('\n\n#endif /* end of SOS_COFF_H_ */\n\n')
    f_coff.close()
    
#plot
fig, (ax1, ax2, ax3, ax4) = plt.subplots(4)

color = 'tab:blue'
ax1.set_title('Frequency response analysis')
ax1.set_ylabel('Gain [dB]')
ax1.set_xlabel('Frequency [Hz]')
ax1.plot(f, gain_db, color=color)
ax1.set_xlim(freq_plot_min, freq_plot_max)
ax1.set_ylim(gain_plot_min, gain_plot_max)
ax1.yaxis.set_major_locator(ticker.MultipleLocator(gain_plot_step))
ax1.grid(which='major', color='g', linestyle='-', linewidth = 0.5)
ax1.grid(which='minor', color='g', linestyle='--', linewidth = 0.25)

color = 'tab:orange'
ax2.set_ylabel('Phase ' + '$[^o]$')
ax2.set_xlabel('Frequency [Hz]')
ax2.plot(f, phase_degree, color=color)
ax2.set_xlim(freq_plot_min,  freq_plot_max)
ax2.set_ylim(phase_plot_min, phase_plot_max)
ax2.yaxis.set_major_locator(ticker.MultipleLocator(phase_plot_step))
ax2.grid(which='major', color='g', linestyle='-', linewidth = 0.5)
ax2.grid(which='minor', color='g', linestyle='--', linewidth = 0.25)

color = 'tab:red'
ax3.set_title('Group delay')
ax3.set_ylabel('Delay [samples]')
ax3.set_xlabel('Frequency [Hz]')
ax3.plot(f, group_delay, color=color)
ax3.set_xlim(freq_plot_min,  freq_plot_max)
ax3.grid(which='major', color='g', linestyle='-', linewidth = 0.5)
ax3.grid(which='minor', color='g', linestyle='--', linewidth = 0.25)

ax4.set_title('Impulse response analysis')
ax4.plot(impuls_sos)
ax4.set_ylabel('Amplitude')
ax4.set_xlabel('Samples')
ax4.set_xlim(0, n_sumple_impuls)
ax4.set_ylim(-0.25, 0.25)
ax4.yaxis.set_major_locator(ticker.MultipleLocator(0.05))
ax4.grid(which='major', color='g', linestyle='-', linewidth = 0.5)
ax4.grid(which='minor', color='g', linestyle='--', linewidth = 0.25)

fig.subplots_adjust(left=0.05, bottom=0.05, right=0.975, top=0.95, hspace=0.5)
plt.show()
