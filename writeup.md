## Writeup Template

### You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---

**Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image1]: ./examples/example0.png "example0"
[image2]: ./examples/example1.png "example1"
[image3]: ./examples/example2.png "example2"
[video1]: ./output/out_project_video_monitor.avi "MonitorVid"
[video2]: ./output/out_project_video.avi "OutputVid"


## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Disclaimer

Dear Mentor,

You may noice I'm writting C++ instead of python. My apology for this unintention I cause. Since I believe became a SDC engineer the foremost thing is writing good C++. So I decide to grasp any chance to harness my C++. Sorry again if this cause any inconvenience.
Please kindly refer to my writeup for how I complete this project and my source code is under `src/` directory.

P.S: the video under `output/` has visualize most of the step I've done.

### Camera Calibration

#### 1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.

I compute camera matrix and distortion coefficients by first collect the 2D coordinates of corners from list of chessboard images provided by invoking `findChessboardCorners` and initialize the corresponding 3D points.
Next using 2D points and corresponding 3D points to compute camera matrics and distortion coefficients by invoking `calibrateCamera`.
This part is written under `LaneDetector::cameraCalib`.

Lastly, using camera matrix and distortion coefficients to undistort raw input image.
This part is written under `LaneDetector::undistImage`.

![alt text][image1]

### Pipeline (single images)

#### 1. Provide an example of a distortion-corrected image.

To demonstrate this step, I will describe how I apply the distortion correction to one of the test images like this one
see following image row 1 and column 2
![alt text][image2]

#### 2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.
I use R (threshold: 200) channel of RGB space and S (threshold: 170) channel of HSL space and use gradient in x direction (threshold 20-100) to conduct color transform. In the threshold binary image, R, S, dx thresholded parts are visualized as R, G, B color respectively.
This part is written under `LaneDetector::thresholding`.

see following image row 1 and column 3
![alt text][image2]

#### 3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.
I use following src and dst points (by eye balling):
This resulted in the following source and destination points:

| Source        | Destination   |
|:-------------:|:-------------:|
| 250, 720      | 320, 0        |
| 595, 450      | 320, 720      |
| 687, 450      | 960, 720      |
| 1170, 720     | 960, 0        |

and pass them into `getPerspectiveTransform` to get perspective transformation matrices and its inverse counterpart.
I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image. (see ...)
This part is written under `LaneDetector::projectForward` and `LaneDetector::projectBackward`.

see following image row 2 and column 1
![alt text][image2]

#### 4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

At the first iteration,

I use bottom half of warped thresholded binary image to compute the summation of pixel intensity along vertical axis and find the peak of left part and right part of image respectively as the initial detection of left and right line position.

Then I applied sliding window along vertical axis bottom-up with width of 200 and mean of prior detection as center to search for activated pixels and store them in an array. Next, using all activated pixel found to fit a second-order polynomial using `PolynomialRegression` defined under `PolynomialRegressional.hpp` and return three coefficient.

At the next iteration,

I use previously fitted polynomial as the center and span left and right by margin as the search space to find activated pixel. And repeat the fitting followed by update.

Above part is written under `LaneDetector::slidingWindow`.

see following image row 2 and column 1
![alt text][image2]

#### 5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

For computing radius:
I presume the length of the lane projected into "bird-eye" view is 30m. And the image height in pixel is 720 px. So the ration of meter to px is `ym_per_px = 30/720.`
Then I leverage the polynomial coefficients and forumla of `R(curve) = (1 + (2A*y*ym_per_px + B)^2)^1.5 / |2A|` to compute the Radius of left and right line.
Since left and right line should be parallel. So I simply average the radius between left and right line's to get final radius.

Above part is written under `LaneDetector::compCurveRadius`


For computing the position of the vehicle w.r.t center:
I use the detected x position of left and right line at the bottom of warped image to compute the center of the lane in warped image.
Then I use computed inverse transformation matrice to project the center back to front-view image.
Then I compute the distance between the warped-back center of the line and front-view image center and times the `xm_per_px=3.7/700`(defined by eyeballing) to convert the distance into meter.

Above part is written under `LaneDetector::compDistToCtr`


#### 6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

see following image row 2 and column 2 and 3
![alt text][image3]

---

### Pipeline (video)

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

Here's a video collage for different stage of visualization: https://youtu.be/LEO-nsC6jlA

Here's a video of final output: https://youtu.be/4H7pn6l1hLI

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

The glare will cause the problem as shown in harder\_challenge\_video and also the color changing of lane segment as shown in project\_video will cause gradient being activated resulting in false detection. Last but not least, the oculusion of lane line by grass, leaves as shown in harder video is also a very big challenge.

I suggest to use semantic segmentation deep model (such as https://github.com/mapillary/inplace\_abn) which is trained in Mapillary dataset and enabled lane line detection to intervene whenever traditional approach report low confidence.
