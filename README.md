# Cell-Top-Monitor
An Open-Source monitor for lithium cells

This is based on a 32bit ARM M0+ processor from ST, The STM32F030F4.
Boards have been designed to suit terminal spacings of 64 and 81mm which suits many 40Ah to 100Ah cells, and a 106mm spacing to suit 200Ah cells. The monitor is intended for LiFePO4 chemistry Sinopoly cells but can be easily adapted to other manufacturers cells and chemistries.

Operation.
The system spends most of it's time with the clock stopped consuming a few microamps of current.
Every 2 seconds it wakes up and takes a reading of the cell voltage and temperature (measured at the -ve terminal).
The sampling process takes around 200us.
If the voltage is above a threshold it turns on a balancing load of around 1.2 Amps.

This is part of a full Open-Source BMS project located at https://github.com/DuncanAitken/OSBMS
