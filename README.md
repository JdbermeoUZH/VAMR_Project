# VAMR_Project
Class Project



How to run:
 - in src/setup/loadfilepaths , modify the pathe of the datasets to match yours
 - in src/setup/LoaHyperParams, set the number of frames to test on, as well as parameters of choice
 - Run main_paul_Testing

It is currently set to work on Malaga, on which results are ... interesting








## File structure:

Note: the ```_``` character here denotes to being a directory and not the actual directory name

```
+-- _src
|   +-- _featureDetection
|      [our differnet implementations for featrue detection]
|   +-- _utils
|       [plotting, debugging, etc]
|   +-- _setup
|       [loading global variables, hyperparams, ...]
|   +-- main (where all other modules are called)
+-- _datasets
|   +-- _kitti
|   +-- _malaga
|   +-- _parking
```
