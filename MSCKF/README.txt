Use the CMakeLists to create a makefile which can be used to build a demonstration of the implementation.

IMPORT : in order to be able to compile the code, you need to have OpenCV and Eigen3 installed on your computer

The binary file will be built in the "build" folder.

At this stage, you can execute the demo, all the necessary files are provided in the folder.

If you want to run the demo on your own dataset, you have to provide the following files:
    * cameraMatrix.yml  : A file containing the intrisic parameters of the camera
    * distCoeffs.yml    : A file containing the distortion parameters of the camera
    * data.yml          : A file containing the flight log of IMU measurements at 100z
    * parameters.yml    : A file containing the parameters required for the initialisation of the MSCKF
    * flight_video.avi  : A video file recorded during the flight (framerate 30Hz)
    
Note that the video file and the flight log should begin at the same time instant.
The camera calibration parameters have the same format than the one used by opencv.
