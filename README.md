# MIDI IMPLEMENTATION - V2.0

This repositoty includes software and hardware design. Anyway just the software part is modified here to support MIDI. 

It is synchronised with V4.5.0 from  GaudiLabs/OpenThereminV4. 

### New : Improved MPE Compatibility
Version 2.0 of MIDI SW has been improved to better support synthesizers compatible with MIDI Polyphonic Expression (MPE) technology (Support of monophonic aftertouch, list of CC for volume loop more in line with specific musical gestures of MPE instruments, first startup comfiguration compatible with MPE). MPE offers many advantages for the MIDI theremins (see "First user experience..." later in this readme file).   

### New : Save your favourite configuration
Version 2.0 and higher also provides the capacity to save your favourite startup configuration in EEPROM so as it persits after power OFF/ON cycle (see "Calibration and configuration saving" later in this document). 

# OpenThereminV4
Open Source Theremin Instrument

Schematics, printed circuit board (PCB) design, bill of materials (BOM) and Arduino UNO compatible software with MIDI support for the OpenTheremin V4

### Open Source Theremin based on the Arduino Platform

OpenTheremin is an arduino based real Theremin instrument. The legendary music instrument was invented by Leon Theremin back in 1920. The theremin is played with two antennas, one to control the pitch and one for volume. The electronic shield with two ports to connect those antennas comprises two heterodyne oscillators to measure the distance of the hand to the antenna when playing the instrument. The resulting signal is fed into a AtMega328P microcontroller. After linearization and filtering the circuit generates the instruments sound that is then played through a high quality digital analog audio converter on the board. The characteristics of the sound can be determined by a wave table on the arduino.

For more info on the open source project and on availability of ready made shield see:

http://www.gaudi.ch/OpenTheremin/

### MIDI 1.0 standard
MIDI is an acronym for Musical Interface for Digital Instrument. It is a quite old standard dating
back to 1983 that was initially designed to connect together (keyboard) synthesizers. 
MIDI 1.0 standard is implemented here in the open theremin to allow its usesr to control an external syntheziser. 
More information about MIDI and Theremin : https://app.box.com/s/s3yx1ro1v8ay061626wy09vmbo7do23q 

### MPE (MIDI Polyphonic Expression)
MIDI Polyphonic Expression is an attempt to adapt MIDI 1.0 protocol to polyphonic continuous controllers. 
The development of these controllers by the late 2010’s led to the development of software synths that can be interesting to play with a theremin because of their great Pitch Bend Range of 48 semitones allowing long glissandos. 
Somehow, MPE also incidentaly formalized the way to interact with monophonic continuous controlers. 

### Installation
1. Download ZIP file from the green "<> Code" icon and unzip the file in proper directory.  
2. Open up the Arduino IDE (install it from https://www.arduino.cc/)
3. Open the File "Open_Theremin_V4.ino"
4. Important Step !  In "Application.cpp", take care of selecting MIDI mode that correponds to your cituation (put "//" in front off inadequate line - Real MIDI Baudrate is selected by default here):

   //Serial.begin(115200); // Baudrate for midi to serial. Use a serial to midi router https://github.com/projectgus/hairless-midiserial
   
   Serial.begin(31250); // Baudrate for real midi. Use din connection https://github.com/MrDham/OpenTheremin_V4_with_MIDI/blob/main/MIDI_DIN_TO_OTV4.jpg or HIDUINO https://github.com/ddiakopoulos/hiduino

   NOTES: 
   
   * If you look carefully at the schematic, you may notice that the serial controler can no more be reprogramed into a MIDI controler.  
   Anyway, I kept the reference to HIDUINO here, just in case that some users want to add some additional material to make a real MIDI USB interface. 
   
   * Open Theremin being very compact, instead of DIN 5 connector, you might want to use a TRS 2.5 mm (recommanded) or 3.5 mm Jack connector as recently adopted by the MIDI standard: https://www.midi.org/midi-articles/trs-specification-adopted-and-released  
   
5. Select the correct usb port in Tools -> Serial Port
6. Select "Arduino Uno" board in Tools -> Board
7. Upload the code by clicking on the upload button.

### First user experience with Open Theremin (MIDI software version 2.0 or higher) paired with the Surge XT software synthesizer
1. Install SURGE XT from https://surge-synthesizer.github.io/ on your computer
2. Power your theremin, ground it properly, keep default factory settings unchanged.  
3. Connect your theremin's MIDI interface to your computer (if you have a "twin DIN" to USB cable, connect MIDI IN cable to the theremin)
4. Let your theremin warm up and calibrate it (with Pitch and Volume knob at 12 o'clock) 
5. Start SURGE XT and activate the proper MIDI input connected to your theremin in option menu.
6. Configure audio properly in Surge XT (depending on your system)
7. Activate MPE mode
8. Select a fancy preset
9. Play

### How does it work ? 

The MIDI open theremin generates NOTE ON/OFF messages and  Continuous Controler changes (MIDI CC) depending on settings and hands' position next to antennas. 


MIDI CC: 

It is possible to assign independant MIDI CCs to the PITCH ANTENNA (ROD) and to the VOLUME ANTENNA (LOOP).  

NOTE ON/OFF: 

In MIDI standard NOTE ON/OFF messages have a NOTE NUMBER and a VELOCITY. 

Let's consider a Fade-in / Picth Variation / Fade-out sequence (I use right handed convention): 

 1. Fade-In

     When left hand moves away from VOLUME ANTENNA (LOOP) and volume crosses a settable threshold (Volume trigger), a NOTE ON is generated. VELOCITY depends on how fast left hand is moving.
     Right hand's position next to PITCH ANTENNA (ROD) determines the starting NOTE NUMBER. 


 3. Pitch variation

    When right hand moves next to PITCH ANTENNA (ROD), PITCH BEND messages are generated (if activated) to reach exact pitch as long as pitch bend range will do.  Beyond, a new NOTE ON followed by a NOTE OFF for the previous note are generated if legato mode is activated. Pitch bend range can be configured (1, 2, 4, 5, 7, 12, 24 or 48 semitones) to align with synth's maximum capabilities. 

 4. Fade-Out

    When left hand moves close to VOLUME ANTENNA (LOOP) and volume goes under Volume trigger threshold, a NOTE OFF is generated to mute the playing note. 

  
MONOPHONIC AFTERTOUCH: 

It is possible to control Monophonic Aftertouch with the upper range of VOLUME ANTENNA (LOOP). 
It works like the CC assigned to the antenna except that the CC covers the full range of the antenna sensitivity whereas Aftertouch covers the upper 50% range with a full 7 bits definition (from 0 to 127). Thus, aftertouch provides an additional control, allowing you to modulate the sound after the note has been played and has become audible. 

MUTE CAPACITIVE BUTTON: 

Sends current NOTE OFF, ALL NOTE OFF and ALL SOUND OFF messages on selected channel so as no sound survives to "mute order" and keep the theremin in mute state (no sound and no MIDI massages) until it is pushed again. 

### Settings
 
 "Register" pot becomes "Selected Parameter" pot and have 8 positions. 
 
 "Timbre" pot becomes "Parameter's Value" and have a variable number of positions depending on selected parameter: 
 
 1. Register: 3 positions (-1 Octave, center, +1 Octave) as in original Open Theremin V4 

 2. Timbre: 8 positions as in original Open Theremin V4

 3. Channel: 16 positions (channel 1 to 16)

 4. Note Lifecycle: 8 positions 
     Activates Aftertouch (MSB), Legato and PitchBend (LSB) in a 3 bit binary encoding mode.  

 5. Pitch bend range: 8 positions (1, 2, 4, 5, 7, 12, 24, 48 Semitones). 
     For classical glissando and in order to have same note on audio and MIDI, use exactly same pitch bend range on your synth. 
     Maximum setting possible is recomended.

 6. Volume trigger / Velocity sensitivity (how fast moves the volume loop's hand): 128 positions (0 to 127)

    The volume trigger can be configured so as we have some volume at note attack on percussive sounds.

    The volume trigger setting is also used to set sensitivity for velocity (how fast left hand is moving when note is triggered).

    Volume trigger = 127 (Maximum) won't generate any NOTE ON. It can be used to generate MIDI CC only.

    Tweakable parameter in application.cpp: "#define VELOCITY_SENS 9" -> How easy it is to reach highest velocity (127). Something betwen 5 and 12.
    Changing this to your taste may require some test and trial.
     
 7. Rod antenna MIDI CC: 8 positions
   
     None, 8-Balance, 10-Pan, 16-MSB/48-LSB-GeneralPurpose-1, 17-MSB/49-LSB-GeneralPurpose-2, 18-GeneralPurpose-3, 19-GeneralPurpose-4, 74-cutoff

    For 14 Bit CC messages, MSB and LSB are always sent together and in the following order: MSB (1st), LSB (2nd) as per MIDI 1.0 Standard.

    The receiver can bufferize MSB to synchronize it with the LSB.

    In function "Application::set_parameters" of application.cpp you can tweak the code to configure the list of CC associated Rod antenna.

    You can aslo scale the CC sensivity to hand movement (128 corresponding to 1:1 scale).

    Manipulation of Rod antenna MIDI CC is not error proof. It is much easier to change its value in MUTE mode. 
    
 8. Loop antenna MIDI CC: 8 positions

     None, 1-Modulation, 2-breath, 4-Pedal, 7-Volume, 11-Expression, 71-Resonnance, 74-Cutoff

     Before 2.0, the 8 positions used to be 1-Modulation, 7-Volume, 11-Expression, 71-Resonnance, 74-Cutoff, 91-Reverb, 93-Chorus, 95-Phaser. This modification was decided in order to allow "None" option and to give priority to CC numbers that better correspond to a musical gesture than to an effect control (e.g. Breath over Reverb).

     In function "Application::set_parameters" of application.cpp you can tweak the code to configure the list of CC associated Loop antenna.

     Manipulation of Loop antenna MIDI CC is not error proof. It is much easier to change its value in MUTE mode. 
   
Select a Parameter and move "Parameter's Value" to change corresponding setting 

While you rotate the pots, the LED toggles (OFF/PINK) every steps to give you some angular feedback before going back to PLAY/MUTE Status. 
Please look at the quick guide here for more detail: https://github.com/MrDham/OpenTheremin_V4_with_MIDI/blob/main/Antiseche%20open%20theremin%20PnB.pdf 
 
### Default Startup configuration

In order to have a first plug and play user experience with MPE synths (e.g. Surge XT), from version 2.0, default startup configuration is: 

 1. Register = Center 

 2. Timbre = 1st Waveform 

 3. Channel = MIDI Channel 2 

 4. Note Lifecycle = Aftertouch on/Legato on/Pitch Bend on 

 5. Pitch bend range = 48 Semitones

 6. Volume trigger = 0

 7. Rod antenna MIDI CC = None

 8. Loop antenna MIDI CC = 7-Volume. 

Before version 2.0 it used to be: Register = Center, Timbre = 1st Waveform, Channel = MIDI Channel 1, Note Lifecycle = Legato on/Pitch Bend on, Pitch bend range = 2 Semitones, Volume trigger = 0, Rod antenna MIDI CC = None, Loop antenna MIDI CC = 7-Volume


You can save your own favourite startup configuration in EEPROM (see "CALIBRATION AND SAVING STARTUP CONFIGURATION") so as it persists after Power OFF ON cycle. 

### Technical detail about MPE impacting settings

A global principle of MPE is to define one or two zones (called Upper and Lower Zone), 
each one having a Manager Channel (1 or 16) and a group of Member Channels (between Channel 2 and
Channel 15). These Member Channels are purposed to distribute the polyphonic notes when using a
polyphonic controller, each note being taken as a monophonic instrument. This way, each note can
have its own Pitch Bend, its own Aftertouch and its own Continuous Controlers. Hence the name of
“polyphonic expression”.

A common default setting of MPE synths is to have Pich Bend Range of Manager Channel set to 2 (for Pitch wheel control) and
Pitch Bend Range of Member Channels set to 48 (for long glissandos). 

Without entering into too much detail here, the theremin being monophonic, it simply needs to use
one of the Member Channels (e.g. Channel 2 for the Lower Zone) and to have Pitch Bend Range set
to 48 to be able to control properly a MPE synth. 
 

### Audio output 

Audio processing from antennas to output jack, including volume and pitch pots, LEDs and button functions, is exactly the same as in open theremin V4.  You can play the Audio and control MIDI devices side by side. 

### CV GATE 

CV GATE processing from antennas to output jack is the same as in classic open theremin V4. I just increased precision of associated mathematics to follow more accurately audio and MIDI frequency so as you can control your analog devices and MIDI devices side by side. 

### Calibration and configuration saving

You can perform calibration with or without saving your configuration (in both cases antenna are calibrated). If you save calibration, it will persist after Power OFF-ON cycle. 

How to save configuration during calibration:

 1. Turn Volume full left

 2. Turn Pitch full right

 3. Long press Function Button

    *  Wait for end of calibration 

 4. Re-center Pitch & Volume

Keep Pitch and Volume centered to perform calibration without saving configuration. 

### Need some support ? 
Please log bugs, requests and questions at https://github.com/MrDham/OpenTheremin_V4_with_MIDI/issues

### Any questions about MIDI and theremins ?
The answer is probably there: https://app.box.com/s/s3yx1ro1v8ay061626wy09vmbo7do23q 

### Added and removed compare to Open Theremin V4. 
Serial communication implemented for program monitoring purpose was removed (Particularly during calibration).
If you need to monitor calibration for antenna problem fixing, please use original master branch from 
https://github.com/GaudiLabs/OpenThereminV4. 

Serial port is used to send MIDI messages now. 

### Aknowledgement
All this is based on the original project created by Urs Gaudenz with great contributions from Thierry Frenkel. Open theremin V3 and now V4 have been great fun to play since 2017 for me. Thanks !

Many thanks to @JuliusKB (https://github.com/JuliusKB) for test, trial and fruitfull challenge on design during the developement phase. 

