DT0105  
Design tip 
1-point or 3-point tumble sensor calibration 
By Andrea Vitali 
Purpose and benefits 
This design tip explains how to compute offset and gain for a 3-axis sensor (usually an 
accelerometer) by performing a 1 or 3-point tumble calibration. A generalization of the 
algorithm is also described. 
Benefits: 
• Added functionality with respect to calibration provided by the MotionFX library which 
only provides magnetometer and gyroscope calibration and not accelerometer 
calibration. 
• Short and essential implementation which enables easy customization and 
enhancement by the end-user (MotionFX is available only in binary format, not as 
source code) 
• Easy to use on every microcontroller (MotionFX can only be run on STM32) 
Scope 
This design tip applies to all accelerometers, eCompass modules, and iNemo inertial IMUs 
from STMicroelectronics. 
Specifications 
Algorithm specifications: 
• Input from 3-axis sensor: [x,y,z] data triplets for each position 
• Output of 1-point tumble calibration: offset for each axis (Xofs, Yofs, Zofs) 
• Output of 3-point tumble calibration: offset for each axis (Xofs, Yofs, Zofs) and gain for 
each axis (Xgain, Ygain, Zgain)  
January 2022 
DT0105 Rev 2 
1/5 
www.st.com 
Algorithm description for 1-point tumble calibration 
The algorithm is described for the particular case of an accelerometer. However, it can also 
be used with other sensors, e.g. a magnetometer. See notes at the end of this document. 
It is assumed that the sensor has nominal sensitivity equal to 1 for each axis and there is 
no cross-axis sensitivity. True acceleration is related to measured acceleration as follows: 
�
𝐀
𝐀𝐀𝐀𝐀𝐀𝐀𝐀
𝐀
𝐀𝐀𝐀𝐀𝐀𝐀𝐀
𝐀
𝐀𝐀𝐀𝐀𝐀𝐀𝐀
� = �
𝟏
𝟏 𝟎𝟎 𝟎𝟎
𝟎
𝟎 𝟏𝟏 𝟎𝟎
𝟎
𝟎 𝟎𝟎 𝟏𝟏
� �
𝐭
𝐭𝐭𝐭𝐭𝐭𝐭𝐭𝐀𝐀𝐀𝐀𝐀𝐀𝐀𝐀
𝐭
𝐭𝐭𝐭𝐭𝐭𝐭𝐭𝐀𝐀𝐀𝐀𝐀𝐀𝐀𝐀
𝐭
𝐭𝐭𝐭𝐭𝐭𝐭𝐭𝐀𝐀𝐀𝐀𝐀𝐀𝐀𝐀
� +�
𝐀
𝐀𝐗𝐗𝐗𝐗𝐗𝐗
𝐀
𝐀𝐗𝐗𝐗𝐗𝐗𝐗
𝐀
𝐀𝐗𝐗𝐗𝐗𝐗𝐗
�  
The sensor should be oriented so that only one axis (e.g. Z = 1g) is stimulated while the 
others are orthogonal to the stimulus (e.g. X=Y=0g): trueAcc =  [0, 0, +1] 
Measured acceleration, derived from the equation shown at the beginning, is calculated by 
plugging in the values listed above for true acceleration: 
1. AccX =  0 + Xofs,   Xofs = AccX 
2. AccY =  0 + Yofs,   Yofs = AccY 
3. AccZ =  1 + Zofs,   Zofs = AccZ-1 
Offsets are readily available and can now be subtracted to go from measured acceleration 
to true acceleration. 
It must be noted that if the sensitivity is not 1 as assumed, or if cross-axis sensitivities are 
not 0 as assumed, the computed offset will be wrong. Also if the true acceleration is not  
[0, 0, 1] during calibration, the computed offset will be wrong. This can happen for non-ideal 
sensors and imperfect alignment during calibration. 
Algorithm description for 3-point tumble calibration 
The algorithm is described for the particular case of an accelerometer. However, it can also 
be used with other sensors, e.g. a magnetometer. See notes at the end of this document. 
It is assumed that the sensor has no cross-axis sensitivity. True acceleration is related to 
measured acceleration as follows: 
�
�
𝐀𝐗𝐗𝐗𝐗𝐗𝐗𝐗𝐗
�
𝐀
𝐀𝐀𝐀𝐀𝐀𝐀𝐀
𝐀
𝐀𝐀𝐀𝐀𝐀𝐀𝐀
𝐀
𝐀𝐀𝐀𝐀𝐀𝐀𝐀
� = �
𝟎
𝟎
𝟎
𝟎
𝟎
𝟎
𝐀
𝐀𝐗𝐗𝐗𝐗𝐗𝐗𝐗𝐗
𝟎
𝟎
𝟎
𝟎
𝟎
𝟎
𝐀
𝐀𝐗𝐗𝐗𝐗𝐗𝐗𝐗𝐗
� �
𝐭
𝐭𝐭𝐭𝐭𝐭𝐭𝐭𝐀𝐀𝐀𝐀𝐀𝐀𝐀𝐀
𝐭
𝐭𝐭𝐭𝐭𝐭𝐭𝐭𝐀𝐀𝐀𝐀𝐀𝐀𝐀𝐀
𝐭
𝐭𝐭𝐭𝐭𝐭𝐭𝐭𝐀𝐀𝐀𝐀𝐀𝐀𝐀𝐀
� +�
𝐀
𝐀𝐗𝐗𝐗𝐗𝐗𝐗
𝐀
𝐀𝐗𝐗𝐗𝐗𝐗𝐗
𝐀
𝐀𝐗𝐗𝐗𝐗𝐗𝐗
�  
The sensor should be oriented so that only one axis at a time is stimulated (e.g. X, then Y, 
then Z) while the others are orthogonal to the stimulus. 
True [x,y,z] acceleration for each position in a 3-point tumble calibration is as follows: 
1. Gravity vector along +X axis: trueAcc =  [+1,  0,   0 ] 
2. Gravity vector along +Y axis: trueAcc =  [  0, +1,  0 ] 
3. Gravity vector along +Z axis: trueAcc =  [  0,    0, +1] 
January 2022 
DT0105 Rev 2 
2/5 
www.st.com 
Measured acceleration for each position in a 3-point tumble calibration, derived from the 
equation shown at the beginning, is calculated by plugging in the values listed above for 
true acceleration: 
1. AccX1 =  Xgain+Xofs,   AccY1 =     0      +Yofs,   AccZ1 =    0      +Zofs 
2. AccX2 =     0     +Xofs,    AccY2 =  Ygain+Yofs,   AccZ2 =    0      +Zofs 
3. AccX3 =     0     +Xofs,    AccY3 =    0      +Yofs,   AccZ3 =  Zgain+Zofs 
Offsets can be taken directly from the measurements listed above or they can be computed 
by averaging two out of three measurements listed above. Averaging can be used to 
improve the quality of the final estimate. 
• AccX2+AccX3 = 2 Xofs,  Xofs = (AccX2 + AccX3)/2 
• AccY1+AccY3 = 2 Yofs,  Yofs = (AccY1 + AccY3)/2 
• AccZ1+AccZ2 = 2 Zofs,   Zofs = (AccZ1 + AccZ2)/2 
Once offsets are computed, gains can be computed as follows: 
• Xgain = AccX1 - Xofs 
• Ygain = AccY2 - Yofs 
• Zgain = AccZ3 - Zofs 
Now offsets can be subtracted, and multiplication by inverse gains can be done to go from 
measured acceleration to true acceleration. 
Notes 
Applications to other sensors: 
• While it may be easy to exploit the known gravity vector to impose the desired true 
reference on the accelerometer, it may not be possible to achieve perfect accuracy 
when switching from one position to another during the 3-point tumble calibration 
• An alternative way to perform the calibration is to use a gold reference sensor 
which has the same orientation of the sensor to be calibrated, so that the desired 
true reference can be measured and checked. 
• Special equipment, arrangements or procedures may be needed to impose or 
measure the desired true reference on other sensors such as magnetometers or 
gyroscopes 
• For the case of the magnetometer: Helmholtz coils can be used to impose the 
desired true reference; alternatively, the Earth’s magnetic field vector can be used 
together with a gold reference sensor. 
January 2022 
DT0105 Rev 2 
3/5 
www.st.com 
• For the case of the gyroscope:  a single-axis turn table or a step-motor spin table 
can be used to impose the desired true reference; alternatively a gold reference 
sensor can be used as previously described. 
Other algorithms: 6-point tumble calibration, discussed in Design Tip DT0053, can be used 
to estimate offsets, gains and cross-axis gains. In the same Design Tip, a generalization is 
also presented for N-point tumble calibration. 
Support material 
Related design support material 
BlueMicrosystem1, Bluetooth low energy and sensors software expansion for STM32Cube 
Open.MEMS, MotionFX, Real-time motion-sensor data fusion software expansion for STM32Cube 
Documentation 
Application note, AN4508, Parameters and calibration of a low-g 3-axis accelerometer 
Application note, AN4615, Fusion and compass calibration APIs for the STM32 Nucleo with 
the X-NUCLEO-IKS01A1 sensors expansion board 
Design Tip, DT0053, 6-point tumble sensor calibration 
Revision history 
Date 
Version 
28-Aug-2018 
Changes 
1 
Initial release 
21-Jan-2022 
2 
Updated “Purpose and benefits” and “Specifications” on 
page 1 
January 2022 
DT0105 Rev 2 
4/5 
www.st.com 
IMPORTANT NOTICE – PLEASE READ CAREFULLY 
STMicroelectronics NV and its subsidiaries (“ST”) reserve the right to make changes, corrections, enhancements, 
modifications, and improvements to ST products and/or to this document at any time without notice. Purchasers should 
obtain the latest relevant information on ST products before placing orders. ST products are sold pursuant to ST’s terms and 
conditions of sale in place at the time of order acknowledgement. 
Purchasers are solely responsible for the choice, selection, and use of ST products and ST assumes no liability for 
application assistance or the design of Purchasers’ products. 
No license, express or implied, to any intellectual property right is granted by ST herein.  
Resale of ST products with provisions different from the information set forth herein shall void any warranty granted by ST for 
such product. 
ST and the ST logo are trademarks of ST. For additional information about ST trademarks, please refer to 
www.st.com/trademarks. All other product or service names are the property of their respective owners. 
Information in this document supersedes and replaces information previously supplied in any prior versions of this document. 
© 2022 STMicroelectronics – All rights reserved 
January 2022 
DT0105 Rev 2 
5/5 
www.st.com 