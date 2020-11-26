# Hand-detection-app
Hand detection application written as my master thesis project

The application is a part of my master thesis project called "Project of a hand controlled using a vision system".
The application consists of two main parts:

1) Images processing app (folder: "Master_thesis_software") written i C++ with OpenCV in MS Visual Studio 2017. The app allows to detect hand gesture or to calculate an angle
of rotation in wrist. More details about working of the app are included in its main.cpp file.
2) Microcontroler app (folder: "MCU_app") which is responsible for analyze data received from images processing app and for control motors of the real hand model (this model was
made as a mechanical subsystem of the entire project). MCU app was written in Eclipse IDE, MCU configuration was done using STM32CubeMX. Used MCU is STM32F411VET on
STM32F411E-Disco.

Folder "Results" contains example images which are results of hand gesture recogntion (calibration image: ToCalib_fingers.jpg , detection image: ToDetect_3.jpg). In the folder you can
find also photos of real model after this gesture recogntion.
