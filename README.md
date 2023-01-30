# budget-gpsdo
Provable 10MHz±0.01Hz (or better) into 50Ω at low cost 

The original intent of this project was to create a minimum cost GPSDO using a microprocessor to do as much of the work as possible. The design goal was 10MHz within 1 part per billion or better. Two other goals were to minimise user effort (no fiddly parameters to change) and have the GPSDO able to estimate if it was meeting the accuracy goal.

An initial design was created, and met the goals if the user was patient. The oscillator was easily perturbed when devices were attached, and required time to settle. The problem was thought to be unwanted variations in voltages applied to the OCXO. Apart from that failing, the design proved satisfactory. It consisted of a cheap used OCXO - the OSC5A2B02, a buffer 74HC04, and a controller PIC16F1455 supplied by a 5V wall wart.

To rectify the problem, a redesign isolated the OCXO from the influence of other components. The power supply was changed to a nominal 12V delivered to a buck converter delivering about 7V. This then supplies two regulators, one to supply the OCXO and one to supply the remaining components. The control voltage is buffered at the digital level with complementary MOSFETs (2N7000, BS250) so the analog control voltage is derived from the OCXO supply, not the PIC16 supply. To provide further isolation, the optional serial interface to the PIC16 is via optocouplers. This naturally increases the final cost, but this is justified by the improved stability.

To achieve the goal of minimal cost, there are several unusual features.

1 - the control voltage is generated by the PICs 10-bit PWM, the granularity is improved by dithering the pulses on a pulse by pulse basis. To interpolate between raw PWM values, selected pulses are extended by 25ns. The filtered output is equivalent to a 24 bit DAC.

2 - the PIC16 uses the OCXO output as its clock. That allows the PIC peripherals to time the arrival of 1 pulse per second from the GPS module to the nearest 25ns. There is no need for additional circuitry.

3 - the control voltage is changed at discrete intervals, a balance between the second to second variability of the GPS and the long term drift of the OCXO. This allows an estimate to be made of the accuracy of the 10MHz output.

This is a working project. Included here (when I get around to it) will be the circuit, PCB design, construction, commissioning and run documentation.

Although the design uses low cost components, it should translate to a high performance design by using a timer grade GPS module and a higher specification OCXO. The software is self calibrating so no changes would be needed.