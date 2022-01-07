# VAMR_Project
Class Project

Scripts Tom messed with:
mainPaul , baselineTest, Baseline, ValidateKeyframe, matsplit, linearTriangulation
should check disambiguateRelativePose as well because of the whole mirror thing

and you need to remove contOp from the path because it destroys my soul








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
