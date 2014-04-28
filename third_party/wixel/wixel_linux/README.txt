Pololu Wixel Linux Software

Architecture: amd64
Release Date: 2011-06-23
http://www.pololu.com/


== Summary ==

This binary release contains the Pololu Wixel Configuration Utility
(wixelconfig) and the Wixel command-line utility (wixelcmd).
These programs allow you to load apps onto the Wixel over USB.


== Prerequisites ==

In Ubuntu, you can install all the prerequisites by running:

  sudo apt-get install libusb-1.0-0-dev libqtgui4

The commnand-line utility wixelcmd only depends on libusb-1.0.
The Pololu Wixel Configuration Utility depends on libusb-1.0 and
on the QtGui library and its dependencies (including QtCore).


== USB Configuration ==

You will need to copy the file 99-pololu.rules to /etc/udev/rules.d/
in order to grant permission for all users to use Pololu USB devices.
If you already plugged in a Pololu USB device, you should unplug it at
this point so the new permissions will get applied later when you plug
it back in.


== Running the programs ==

You can run the programs by typing one of the following commands:

   ./wixelconfig
   ./wixelcmd


== Licensing Information ==

For licensing information, see license.htm


== More Information ==

For more information, see the Pololu Wixel User's Guide:
http://www.pololu.com/docs/0J46
