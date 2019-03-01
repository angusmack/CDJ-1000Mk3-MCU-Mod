# Overview

![alt text](https://github.com/angusmack/CDJ-1000Mk3-MCU-Mod/Images/cdj_mod_view1.png "CDJ1000Mk3 MCU Mod")

This is a modification to the CDJ-1000mk3 using an MCU, which allows SD-card playback of WAV files.

This firmware is for the F7 chip on the 32F746G DISCOVERY board.

This project was originally started by Andrei (DJ Greeb) in the [https://github.com/djgreeb/CDJ-1000mk3_new_life_project](repo).

This mod supports some Rekordbox functions, displaying of static and dynamic waveforms, audio interpolation process. 

I (Angus) have taken over the support of hosting the source code (that Andrei shared recently) and sharing the other supporting files needed such as STL and PCB for creating the finished product 

#Building the source
Use IDE uVision Keil ARM-MDK Professional and open project file F7.uvprojx in MDK_ARM folder.

Make sure you cut the tracks ULPI_1 and 13 as specified in the build manual before flashing.

#To do:
- AIFF support
- MP3 support
- USB playback
- Hot cues
- ?
