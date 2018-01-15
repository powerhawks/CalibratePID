# CalibratePID
Power Hawks program to calibrate PID code

To calibrate P control-loop:
1. Initialize code on robot with .01 P-value and 0 for I and D
2. Double P until oscillation begins, then halve it
3. Add a quarter of P and then an eighth is no oscillation occurs.

To calibrate PI control-loop:
1. Follow steps for calibrating P control-loop
2. Repeat steps but with I-value instead.

To calibrate PID control-loop:
1. Follow steps for calibrating P and I control-loops
2. Repeat steps but with D-value instead.
