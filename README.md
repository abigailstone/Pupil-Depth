# Pupil-Depth

Recording tool for Pupil Labs gaze tracker and Intel RealSense depth camera.


## Usage 

**Note**: Pupil Capture must be open (running in the background) for this script to work properly. If Capture is open, the script will handle the rest.
 
 Run ```python3 capture.py``` to begin the recording process. Enter the participant ID and distance being measured when prompted.

 For a detailed list of options run: 
 
 ```python3 capture.py -h```

 ## Environment 
 To create a Conda environment with all the necessary dependencies, run 

 ```conda env create --name eye --file environment.yml``` 
