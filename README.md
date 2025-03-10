# budget-gpsdo

*** BECAUSE I HAVEN'T LEARNED TO USE GitHub EFFECTIVELY, LATEST INFORMATION IS AT https://sourceforge.net/projects/minimal-gpsdo/ ***

Provable 10MHz±0.01Hz (or better) into 50Ω at low cost 

Files:
Main.asm - the software written in PIC assembler. Read SettingUpMPLAB.pdf to create the .hex file.

SettingUpMPLAB.pdf - setting up MPLAB X V5.35 (the last version to support MPASM).

Software.pdf - an overview of how the software works.

GPSDO-2023-02-12_163841.zip - KiCad 6.0 files with schematic and PCB layout

Circuit.png - a printout of the schematic from KiCad 6.0 (to save you having to set it up)

GPSDOnotesV1.pdf - notes about commissioning and running the GPSDO

Notes: The schematic (and therefore the PCB) has a few optional bits.
1. There are spaces for filter capacitors around the buck converter. Some converters require them. Some have them onboard so not needed on the PCB.
2. There is provision for a precision regulator to supply 5V to the FET pair that generate the control voltage. It and associated components can be left out, a wire link can get the 5V from the OCXO regulator, with only a slight (hard to measure) degradation of performance.
3. There is provision to acquire the NMEA data and 1pps from a remotely located GPS module via a line driver/receiver pair. This has been tested to 20 metres, but most people will get satisfactory results plugging the GPS unit into the PCB. In the testing, a socket was installed for the line receiver (a UA9639). Pulling the IC allowed the GPS to be plugged in directly without removing other components.
4. Use of optocouplers is not strictly necessary but does ensure no earth loops with whatever is connected to the GPSDO.

Latest Update:
2023-04-13 - Latest source, hex, notes in Release V1.2 *** Out of date - see SourceForge for latest ***
