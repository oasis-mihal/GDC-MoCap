# GDC-MoCap
Optical MoCap system built using OpenCV for Game Design Carleton

# Setup
Download the OpenCV 4.10 installer and run it

Go into your system path (Windows Key > type "path" and click Edit the system environment variables)

Click environment variables.
Under the **system** variables click Add...
Set variable name to OPENCV_DIR
Set variable value to C:\Program Files\opencv\build\x64\vc15 or wherever else you installed OpenCV
Save your variables

# Linking to Visual Studio
Copy opencv_world410.dll from C:\Program Files\opencv\build\x64\vc15 or wherever else you installed OpenCV to GDC-MoCap\x64\Release 

Copy opencv_world410d.dll from C:\Program Files\opencv\build\x64\vc15 or wherever else you installed OpenCV to GDC-MoCap\x64\Debug 

The property pages should already be set up correctly

Set the project to **Release** mode from Debug and set x86 to **x64** if applicable.

Hit Local Windows Debugger to confirm the program builds and runs

