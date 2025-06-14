# https://nehajirafe.medium.com/using-fft-to-analyse-and-cleanse-time-series-data-d0c793bb82e3
import numpy as np
from scipy.signal import butter,filtfilt
import matplotlib.pyplot as plt





# Lets start by creating a signal
import numpy as np
# We will use the python scipy library to calculate FFT and then extract the frequency and amplitude from the FFT,
from scipy import fftpack
import pandas as pd

# We will use Butterworth Low Pass Filter , details can be found here , the cutoff frequency will be the peak_frequency[0]
from scipy.signal import butter,filtfilt,square


def generate_signal(time_array):
    # Generate Signal
    signal_freq = 2 # Signal Frequency
    signal_amplitude = 10 # Signal Amplitude
    #Sine wave Signal
    
    # signal = signal_amplitude*np.sin(2*np.pi*signal_freq*time_array)
    signal = signal_amplitude*square(2*np.pi*signal_freq*time_array)
    # The code will generate a 2 hz signal with amplitude of 10


    # Now lets add some 50hz noise with amplitude of 3 to the above signal and create a new Signal
    # Lets add some noise to the Signal
    noise_freq = 50 # Noise Frequency
    noise_amplitude = 3 # Noise Amplitude
    #Sine wave Noise
    noise = noise_amplitude*np.sin(2*np.pi*noise_freq*time_array) #wave
    # Generated Signal with Noise
    noisy_signal = signal + noise

    return signal, noisy_signal



def apply_fft(noisy_signal, time_array, time_interval, total_time):
# FFT to decompose Signal

    sig_noise_fft = fftpack.fft(noisy_signal)
    sig_noise_amp = 2 / time_array.size * np.abs(sig_noise_fft)
    # sig_noise_freq = np.abs(fftpack.fftfreq(time_array.size, 3/1000))

    sig_noise_freq = np.abs(fftpack.fftfreq(time_array.size, time_interval))
    # The following plot can be generated by plotting “sig_noise_freq” vs “sig_noise_amp”

    

    # Calculating the Amplitude
    # All the amplitudes are stored in “sig_noise_amp” , we just need to fetch the top 2 amplitudes from the list
    peak_amplitude = pd.Series(sig_noise_amp).nlargest(2).round(0).astype(int).tolist()
    # Output : [10, 3]

    # Calculating the Frequency
    #Calculate Frequency Magnitude
    magnitudes = abs(sig_noise_fft[np.where(sig_noise_freq >= 0)])
    #Get index of top 2 frequencies
    peak_frequency = np.sort((np.argpartition(magnitudes, -2)[-2:])/total_time)
    # print("peak_frequency = ", peak_frequency)
    return sig_noise_freq, sig_noise_amp, peak_frequency, peak_amplitude



def butter_lowpass_filter(data, cutoff, fs, order):
    # print("applied cutoff freq =  " + str(cutoff))
    nyq = 0.5 * fs # Nyquist Frequency
    normal_cutoff = cutoff / nyq
    # print("normal_cutoff = ", normal_cutoff)
    # Get the filter coefficients 
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a,data)
    return y




def apply_filter(time_interval, time_array, noisy_signal, sig):

    #Seconds to generate data for
    # time_interval = 0.01
    # time_interval = time_array[1] - time_array[0]
    # print("time_interval = ", time_interval)
    total_number_of_data_sample = len(time_array)
    # print("total sampe = ", total_number_of_data_sample)
    total_time = time_interval*total_number_of_data_sample

    ## =========================================


    # FFT to decompose Signal
    sig_noise_freq, sig_noise_amp, peak_frequency, peak_amplitude = apply_fft(noisy_signal, time_array, time_interval, total_time)
    # The following plot can be generated by plotting “sig_noise_freq” vs “sig_noise_amp”
    # plt.figure("FFT")
    # plt.plot(sig_noise_freq, sig_noise_amp)



    # Filter requirements.
    # fs = 100.0       # sample rate, Hz
    fs = 1/time_interval       # sample rate, Hz
    # print("fs = ", fs)

    
    cutoff = peak_frequency[0]     # desired cutoff frequency of the filter, Hz ,      slightly higher than actual 2 Hz
    # print(sig, " peak_frequency = ",  peak_frequency)
    # print(sig, " peak_amplitude = ", peak_amplitude)
    print(sig, "suggest cutoff freq = ",  cutoff)


    # cutoff =2
    # cutoff = cutoff*50
    # if (cutoff <= 1):
    #     cutoff = 1
    cutoff = cutoff*25
    if (cutoff <= 0.5):
        cutoff = 0.5
    # elif(cutoff >= 0.5 and cutoff <= 1):
    #      cutoff = 1
    elif (cutoff >= 4.3):
        cutoff = 4.3
    print(sig, "applied cutoff freq =  ", cutoff)


    order = 2      # sin wave can be approx represented as quadratic
        
    # Filter the data, and plot filtered signals.
    filtered_signal = butter_lowpass_filter(noisy_signal, cutoff, fs, order)
    return filtered_signal, sig_noise_freq, sig_noise_amp





