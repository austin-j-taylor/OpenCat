This is a modified version of the OpenCat repository for my own personal needs/testing with Bittle.

## Changes:
- beep()/meow() use digitalWrite instead of analogWrite because I don't thing the dog's buzzer works as expected (maybe it's a passive buzzer, not active, like the cat?)
- Turned off meowing while the battery is disconnected because it's annoying while coding.
- (WIP) Bittle will sing a song on startup


## Plans:
- Make Bittle be able to sing songs in a non-blocking manner
- Be able to control Bittle with a Nintendo Switch Pro Bluetooth Controller by combining him with a Raspberry Pi Zero W