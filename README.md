# Getting a Dataset

Connect to the robot. Run `save_on_keypress.py`, and press the `[r]` or `[s]` key to save an image. Then, reposition the scene, and repeat until you have enough (perhaps 10-20) images. These images will be named `camera_image##.jpeg`.

# Annotating the Dataset

Run `detect_handles.py` in the same directory as your image files. The program will display the HOG descriptors of the image, and your mouse will control a window moving over the screen. *Left click* to mark the current view as a negative example (is not a handle), and *right click* to mark the current view as a positive example (is a handle). When you are done annotating an image, press the `[enter]` key to move on to the next image.

At any point in time, you can press `[l]` to learn a model given the data you've already provided. At that point, the detection algorithm will run whenever you load another image. This can be useful for seeing if you have provided enough data. You can also press `[r]` to return to viewing the first image, named `camera_image0.jpeg`.

To save the annotated data, you can press `[s]`, which will create a file `model.pickle`. *If that file already exists, it will be overwritten!* This should only take a brief moment, and then you can exit the program. You can also load data with that name at the start of the program by pressing `[o]`, but keep in mind that this will overwrite any data currently in the program.

# Realtime Viewing

Connect to the robot, and run `load_model_and_detect_realtime.py`. Make sure your `model.pickle` file is in the same directory.