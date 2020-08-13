## How to Use a USB Flash Drive to Connect AWS DeepRacer to Your Wi-Fi Network

From DeepRacer Developer Documentation Troubleshooting https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-troubleshooting-wifi-connection-first-time.html

To connect an AWS DeepRacer vehicle to your home or office Wi-Fi network using a USB flash drive, you need the following:

* A USB flash drive

* The name (SSID) and password for the Wi-Fi network that you want to join

To connect an AWS DeepRacer vehicle to a Wi-Fi network using a USB flash drive

Plug the USB flash drive into your computer.

Open a web browser on your computer and navigate to https://github.com/aws-samples/aws-deepracer-workshops/blob/master/Car/USB%20stick%20Wi-Fi%20setup/wifi-creds.txt. This link opens a text file named wifi-creds.txt.

The wifi-creds.txt file describes how to connect to connect to a Wi-Fi network.
                    
Save wifi-creds.txt to your USB flash drive. Depending on which web browser you use, the text file might download to your computer and open in your default code editor automatically. If wifi-creds.txt doesn't download automatically, open the context (right-click) menu and choose Save as to save the text file to your USB flash drive.

Warning Do not change the file name.

If wifi-creds.txt isn't already open, open it in a code editor in plain text mode. Some text editors default to rich text (.rtf) instead of plain text (.txt) when the file type isn't specified, so if you are having trouble editing the file, check your settings. If you are using Windows, you can also try to open the file using the Sublime Text application, which you can download for free, or, if you use a Mac, try the TextEdit application, which is pre-installed on most Mac devices and defaults to plain text.

In between the single quotation marks at the bottom of the file, enter the name (SSID) and password of the Wi-Fi network that you want to use. SSID stands for "Service Set Identifier." It is the technical term for the name of your Wi-Fi network.

Note If the network name (SSID) or password contains a space, such as in Your-Wi-Fi 100, enter the name exactly, including the space, inside the quotation marks (''). If there is no space, using quotation marks is optional. For example, the Wi-Fi password, Passwd1234 doesn't contain a space, so using single quotation marks works but isn't necessary. Both SSID and password are case sensitive.

Save the file on your USB flash drive.

Eject the USB drive from your computer and plug it into the USB-A port on the back of the AWS DeepRacer vehicle between the compute battery power button and the rear stanchion.

The USB-A port is located at rear of vehicle between the compute battery power button and the rear stanchion.

Ensure that the AWS DeepRacer is powered on.

Watch the Wi-Fi LED on the vehicle. If it blinks and then changes from white to blue, the vehicle is connected to the Wi-Fi network. Unplug the USB drive.