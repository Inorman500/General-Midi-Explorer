# General-Midi-Explorer
In this lab the goal was to build and program a MIDI (Musical Instrument Digital Interface) controller, the GME(General MIDI Exmploer). A MIDI controller is a device which sends MIDI messages to a synthesizer. In this lab, the synthesizer is a program
running on the PC, Midi OX.

The GME that my group and I created allows the user to "explore" the sounds available in General MIDI. Specifically, the GME
will record General MIDI notes coming into the device and will modify and replay the notes on command. The user will
interact with the GME to control the modifications by changing the amount of light falling on optical sensors. The light levels
on the sensors will modify the sounds that are played back from the GME through the PC.

THhe schematic for the GME can me seen here https://imgur.com/a/7goG53R


The GME has three modes of operation controlled by switches: Record, Playback, Playback with Modification.

Record: When the GME enters recording mode, a new recording is initiated that will overwrite any recording presently stored.
To make the new recording the user will send MIDI notes into the GME from the computer. The GME will store to EEPROM
the sequence of notes that are received and the timing of the notes. The recording mode should keep count of how many notes
have been stored and should stop recording when it reaches its maximum storage capabilities. The recording stops when the
GME leaves recording mode, but the stored notes remain in memory until a new recording is made, even if the device is reset.

Playback: When the GME enters playback mode, the notes will be played back from EEPROM memory with approximately
the same inter-note timing as in their recording. You may configure the program to enforce a minimum time between playing
note on and corresponding note off if the note durations are too short to hear. When the end of the stored recording is reached,
the recording will start playing again from the beginning. If the system is reset or turned on when the playback switch set to on,
the GME should enter playback mode immediately and begin playing the recorded music from the start.

Playback with Modification: If the playback and modify switches are both on, the playback will be modified depending on the
amount of light falling on an optical sensor. The on-chip ADC converts an analog voltage from the photo sensor circuit into a
digital value. The digital value controls the speed of playback in a calibrated way. 
