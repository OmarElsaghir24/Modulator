# Modulator
Designed a modulator using the TM4C123GH6PM microcontroller that can plot sine and cosine waveforms, and constellations by specifying the modulation type and transmission rate in symbols/second. Also implemented a feature to enable RRC filtering to the constellations.

![Modulator Circuit](https://github.com/user-attachments/assets/18c2e403-8c83-4eb5-9aad-48fd84dc6db7)


# Features
* `Waveform plotting`: plots sine (I) and cosine (Q) waveforms by providing the amplitude and frequency, and then generating lookup tables of I and Q values to be used for plotting the waveforms
* `Symbol streaming`: Plots constellations for BPSK, QPSK, 8PSK, and 16QAM by converting strings to groups of data bits for each modulation type.
* `RRC filtering`: applies rrc flitering to the constellations by first utilizing interpolation filtering by inserting an IQ value into an array and then three zeros after it to an array of size 31. It then uses convolution on this array.

# Hardware Components
|                           |
|---------------------------|
| TM4C123GH6PM Tiva Board |
| MAX660 voltage converter |
| MCP4822 SPI Dual DAC chip |
| TLC072 op amp |
| Oscilloscope |
| Signal Generator |
| Spectrum Analyzer |

# Peripherals Used
|        |
|--------|
| SPI |
| UART |
| Timers |
| GPIO |

# Software Features
* `Wrtie DAC x`: writes a value between 0 to 4095 to the MCP4822 chip that converts that value to a voltage between -0.5 and 0.5 volts.
* `Mode x`: selects with modulation type to plot the constellation for.
* `Plot a b c`: plots the sine and cosine waveforms by specifying a (Amplitude) and b (Frequency). Can also enable clipping on the waveforms by providing c(Clipping voltage).
* `Clipping x`: enables or disables clipping on waveforms.
* `Data x`: writes a string of any length to be used for plotting the constellations.
* `Rate x`: provides a frequency to be used with a timer for plotting the constellations at a transmission rate of symbols/second.
* `Start`: begins the process of plotting waveforms or constellations.
* `Stops`: ends the process of plotting waveforms or constellations.
* `rrc x`: enables or disables applying rrc filtering to the modulation constellations.
