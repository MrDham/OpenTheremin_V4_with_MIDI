# MIDI IMPLEMENTATION - Status : Beta 

### Please log bugs, requests and questions at https://github.com/MrDham/OpenTheremin_V4_with_MIDI/issues


This repo includes software and hardware design. Anyway just the software part is modified here. 

### Progress: 
Removed serial communication in preparation to MIDI implementation. 

Ported MIDI from Open Theremin V3 with MIDI V2.6. 

Reached Beta status. Thanks to @JuliusKB (https://github.com/JuliusKB) for test and Trial. 

Re-synchronised with V4.0.1 from  GaudiLabs/OpenThereminV4 


# OpenThereminV4
Open Source Theremin Instrument

Schematics, printed circuit board (PCB) design, bill of materials (BOM) and Arduino UNO compatible software for the OpenTheremin V4

### Open Source Theremin based on the Arduino Platform

OpenTheremin is an arduino based real Therein instrument. The legendary music instrument was invented by Leon Theremin back in 1920. The theremin is played with two antennas, one to control the pitch and one for volume. The electronic shield with two ports to connect those antennas comprises two heterodyne oscillators to measure the distance of the hand to the antenna when playing the instrument. The resulting signal is fed into a AtMega328P microcontroller. After linearization and filtering the circuit generates the instruments sound that is then played through a high quality digital analog audio converter on the board. The characteristics of the sound can be determined by a wave table on the arduino.

For more info on the open source project and on availability of ready made shield see:

http://www.gaudi.ch/OpenTheremin/

### Installation
1. Open up the Arduino IDE
2. Open the File "Open_Theremin_V4.ino"
3. Important Step !  In "Application.cpp", take care of selecting MIDI mode that correponds to your cituation (put "//" in front off inadequate line - MIDI through serial is selected by default here):

   Serial.begin(115200); // Baudrate for midi to serial. Use a serial to midi router http://projectgus.github.com/hairless-midiserial/
  
   //Serial.begin(31250); // Baudrate for real midi. Use din connection https://www.arduino.cc/en/Tutorial/Midi or HIDUINO https://github.com/ddiakopoulos/hiduino

   NOTE: If you look carefully at the schematic, you may notice that the serial controler can no more be reprogramed into a MIDI controler.  Anyway, I kept the reference to HIDUINO here, just in case that some users want to add some additional material to make a real MIDI USB interface. 
   
4. Selecting the correct usb port on Tools -> Serial Port
5. Select the correct arduino board from Tools -> Board
6. Upload the code by clicking on the upload button.

### Added and removed compare to Open Theremin V4. 
Serial communication implemented for program monitoring purpose was removed (Particularly during calibration).
If you need to monitor calibration for antenna problem fixing, please use original master branch from 
https://github.com/GaudiLabs/OpenThereminV4. 

Serial port is used to send MIDI messages now. 

### How does it work ? 

The MIDI open theremin generates NOTE ON/OFF messages and  Continuous Controler changes (MIDI CC) depending on settings and hands' position next to antennas. 


MIDI CC: 

It is possible to affect independant MIDI CCs to the PITCH ANTENNA (ROD) and to the VOLUME ANTENNA (LOOP).  

NOTE ON/OFF: 

In MIDI standard NOTE ON/OFF messages have a NOTE NUMBER and a VELOCITY. 

Let's consider a Fade-in / Picth Variation / Fade-out sequence (I use right handed convention): 

1. Fade-In

   When left hand moves away from VOLUME ANTENNA (LOOP) and volume crosses a settable threshold (Volume trigger), a NOTE ON is generated. VELOCITY depends on how fast left hand is moving. Right hand's position next to PITCH ANTENNA (ROD) determines the starting NOTE NUMBER. 


2. Pitch variation

   When right hand moves next to PITCH ANTENNA (ROD), PITCH BEND messages are generated (if activated) to reach exact pitch as long as pitch bend range will do.  Beyond, a new NOTE ON followed by a NOTE OFF for the previous note are generated if legato mode is activated. Pitch bend range can be configured (1, 2, 4, 5, 7, 12, 24 or 48 semitones) to align with synth's maximum capabilities. 

3. Fade-Out

   When left hand moves close to VOLUME ANTENNA (LOOP) and volume goes under Volume trigger threshold, a NOTE OFF is generated to mute the playing note. 

  
 SETTINGS:
 
 "Register" pot becomes "Selected Parameter" pot and have 8 positions. 
  "Timbre" pot becomes "Parameter's Value" and have a variable number of positions depending on selected parameter: 
 
 1. Register: 3 positions (-1 Octave, center, +1 Octave) as in original Open Theremin V3 (version V3.1)
 2. Timbre: 8 positions as in original Open Theremin V3
 3. Channel: 16 positions (channel 1 to 16)
 4. Rod antenna mode: 4 positions 
     (Legato off/Pitch Bend off, Legato off/Pitch Bend on, Legato on/Pitch Bend off, Legato on/Pitch Bend on)
 5. Pitch bend range: 8 positions (1, 2, 4, 5, 7, 12, 24, 48 Semitones). 
     For classical glissando and in order to have same note on audio and MIDI, use exactly same pitch bend range on your synth. 
     Maximum setting possible is recomended.
 6. Volume trigger / Velocity sensitivity (how fast moves the volume loop's hand): 128 positions (0 to 127)
 7. Rod antenna MIDI CC: 8 positions 
    (None, 8-Balance, 10-Pan, 16-MSB/48-LSB-GeneralPurpose-1, 17-MSB/49-LSB-GeneralPurpose-2, 18-GeneralPurpose-3, 19-GeneralPurpose-4, 74-cutoff) 
    
    For 14 Bit CC messages, MSB and LSB are always sent together and in the following order: MSB (1st), LSB (2nd) as per MIDI 1.0 Standard. 
    The receiver can bufferize MSB to synchronize it with the LSB. 
    
 8. Loop antenna MIDI CC: 8 positions 
    (1-Modulation, 7-Volume, 11-Expression, 71-Resonnance, 74-Cutoff, 91-Reverb, 93-Chorus, 95-Phaser)

Select a Parameter and move "Parameter's Value" to change corresponding setting. 

The picture at https://github.com/MrDham/OpenTheremin_V3_with_MIDI/blob/master/MIDI%20Open%20Theremin%20V3%20HMI.bmp gives an example of possible HMI: on "Value" pot, red lines have 4 positions, grey lines have 5 positions and yellow lines have 8 positions. On "Parameter" pot you see coloured lines indicating which colour to follow for the "Value" pot. 

The Quick Guide at https://github.com/MrDham/OpenTheremin_V3_with_MIDI/blob/master/Quick%20guide%20open%20theremin%20midi.bmp works well with this HMI. Print it on a portrait A4 sheet of paper, plastify it and take it with your theremin everywhere you go... 

The volume trigger can be configured so as we have some volume at note attack on percussive sounds. 
The volume trigger setting is also used to set sensitivity for velocity (how fast left hand is moving when note is triggered). 
Volume trigger = 127 (Maximum) won't generate any NOTE ON. It can be used to generate MIDI CC only.

Manipulation of "Rod antenna MIDI CC" and "Loop antenna MIDI CC" is not error proof. MIDI newbies should be advised to change their value in MUTE mode. 

 
Default configuration is: Register = Center, Timbre = 1st Waveform, Channel = MIDI Channel 1, Rod antenna mode = Legato on/Pitch Bend on, Pitch bend range = 2 Semitones, Volume trigger = 0, Rod antenna MIDI CC = None, Loop antenna MIDI CC = 7-Volume. 


MUTE CAPACITIVE BUTTON: 

Sends ALL NOTE OFF on selected channel and stay in mute until it is pushed again.  

AUDIO: 

Audio processing from antennas to output jack, including volume and pitch pots, LEDs and button functions, is exactly the same as in open theremin V4.  You can play the Audio and control MIDI devices side by side. 

CV GATE: 

CV GATE processing from antennas to output jack is exactly the same as in open theremin V4.  You can control your analog devices and MIDI devices side by side. 

CALIBRATION:

This device runs normal calibration of antennas after pushing button for a few second as per initial project
