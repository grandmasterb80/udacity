SFND 2D Feature Tracking Project by Daniel Baudisch (Resubmission)
==================================================================

Modifcations
------------

  * Correction of FAST detection (a filter based on tracking was applied before due to Copy&Paste from "intermediate" solutions during the course)
    ==> This gives now more keypoints. It did not change the number of matched keypoints at all. This led to the next change.
  * Using BF & Binary & NN (all descriptorTypes but SIFT, which uses BF & HOG & KNN)
    ==> This gives way more tracked keypoints, especially for detector=FAST.
  * Plenty of modifications regardings the output
      * Most output is now commented out
      * Table is now output in MarkDown format to copy&paste the result into this document (see below)

New observations
----------------

In general, detector={FAST, BRISK} and matcher={BRISK, BRIEF, ORB} give good results in #keypoints and #matched keypoints. While detector=BRISK results
about twice as many tracked keypoints, the detector=FAST result in a ten time faster execution time.

***Based on the new results, I choose following detector / matcher combination (in given order):***
1) FAST detector with BRIEF matcher (1348 tracked points, 0.012sec)
2) FAST detector with BRISK matcher (1348 tracked points, 0.021sec)
3) FAST detector with ORB matcher   (1348 tracked points, 0.028sec)


## Number of Keypoints

           |     BRISK |     BRIEF |       ORB |     FREAK |     AKAZE |      SIFT
:---------:|----------:|----------:|----------:|----------:|----------:|----------
 SHITOMASI |      1179 |      1179 |      1179 |      1179 |         - |      1179
    HARRIS |       197 |       197 |       197 |       197 |         - |       197
      FAST |      1491 |      1491 |      1491 |      1491 |         - |      1491
     BRISK |      2762 |      2762 |      2762 |      2762 |         - |      2762
       ORB |      1161 |      1161 |      1161 |      1161 |         - |      1161
     AKAZE |      1670 |      1670 |      1670 |      1670 |      1670 |      1670
      SIFT |      1386 |      1386 |         - |      1386 |         - |      1386

## Number of Keypoint Matches
      
           |     BRISK |     BRIEF |       ORB |     FREAK |     AKAZE |      SIFT
:---------:|----------:|----------:|----------:|----------:|----------:|----------
 SHITOMASI |      1067 |      1067 |      1067 |      1067 |         - |         0
    HARRIS |       168 |       168 |       168 |       168 |         - |         0
      FAST |      1348 |      1348 |      1348 |      1348 |         - |         2
     BRISK |      2508 |      2508 |      2508 |      2326 |         - |        63
       ORB |       950 |      1033 |      1033 |       549 |         - |         1
     AKAZE |      1491 |      1491 |      1491 |      1491 |      1491 |         0
      SIFT |      1248 |      1249 |         - |      1239 |         - |         0

## Time for keypoint detection and keypoint matching for 10 frames
      
           |     BRISK |     BRIEF |       ORB |     FREAK |     AKAZE |      SIFT
:---------:|----------:|----------:|----------:|----------:|----------:|----------
 SHITOMASI |  0.107484 |  0.095182 |  0.111640 |  0.250736 |         - |  0.142831
    HARRIS |  0.105829 |  0.105109 |  0.113242 |  0.308331 |         - |  0.201947
      FAST |  0.021491 |  0.012487 |  0.028873 |  0.190147 |         - |  0.103229
     BRISK |  0.336192 |  0.315105 |  0.382493 |  0.495511 |         - |  0.464918
       ORB |  0.238283 |  0.051769 |  0.125387 |  0.232777 |         - |  0.207384
     AKAZE |  0.318410 |  0.301407 |  0.351459 |  0.476272 |  0.526876 |  0.413504
      SIFT |  0.619682 |  0.618369 |         - |  0.803187 |         - |  1.040087




SFND 2D Feature Tracking Project by Daniel Baudisch
===================================================

Task 1: Data buffer optimization
--------------------------------

No issues; oldest elements (located at the beginning of the databuffer) have to be removed, until the list contains the desired number of elements. ==> usually it is only one element, using the loop, it ensures that even more elements could be added per iteration.



Task 2: Keypoint Detection
--------------------------

  * I use a mapping of string to functions to avoid an extensive if-then-else construct. It can be also extended in the matching* code itself, i.e. I can simply use the new name in MidTermProjectStudent.cpp without the need to add another "else if".
  * Add function by function. Some of the code was already implemented during the preceding lessons and I just reused it. Others were quite straight forward: instantiate the right detector, call the detect function and call it done.
  * At a fist glance, the ORB seems to provide good results, since the tracking of the truck look continuous (even if the truck is not the object of interest).




Task 3: Evaluation of Detectors and Descriptors
-----------------------------------------------

Idea: rectangle is given, we iterate through each point and check if it is within the rectangle.
Remark: Since, each point has a size, we could also check if the circle has overlap with the rectangle, which would include more points. (not done in this exercise, since it was stated that this should not be done in a final code and it is only for educational purpose) ==> one could enlarge the ROI to ensure not to miss some of the points.



Task 4: Implement a variety of keypoint descriptors
---------------------------------------------------

The function descKeypoints in matching2D_Student.cpp was extended by adding more if-then-else to instantiate the correct extractor. The instantiation is straight-forward. Running different extractors resulted in some crashes - not each extractor can be run for each keypoint detector.



Task 5: Keypoint Matching
-------------------------

I took the code developed during the lesson to implement the descripter_matching.cpp. Beside adding the missing parts, I had to update the computation of "int normType" to allow usage of NORM_L2, which is required for some of the combinations to avoid following exception:

terminate called after throwing an instance of 'cv::Exception'
  what():  OpenCV(4.3.0) /home/baudisch/Downloads/opencv/opencv/modules/core/src/batch_distance.cpp:282: error: (-215:Assertion failed) (type == CV_8U && dtype == CV_32S) || dtype == CV_32F in function 'batchDistance'



Task 6: Implement the descriptor distance ratio test
----------------------------------------------------

Is already included in the work of Task 5, i.e. the functionality was already implemented and reused.



Task 7,8,9: Implement the descriptor distance ratio test
--------------------------------------------------------

Instead of executing all the different combinations manually, I added a loop to iterate through all the iterations. I moved the actual code into a new function "runBenchmark", which takes as argument the
detector (string), the matcher (string) and three references to the variables that are used to store the results (#keypoint, #matches, time). For each iteration, the #keypoints, #matches and time for the
key operations is recorded. At the end of the loop, all data is written to three files (for each task one table) in CSV format. I used libreoffice/calc to merge the tables into one to easily check the
results. Already the #keypoints showed that BRISK seems to be able to detect more corners for tracking. The tracking result is pretty clear: BRISK is definitely the corner detector to go. As matcher, all
(but AKAZE, which can only run with AKAZE detector) result in a fairly usable amount of tracking points. Nevertheless, BRISK as matcher gives the highest number of tracking points over all images (around
8 to 9 in average per image).

***Best out of three:***
1) BRISK detector with BRISK matcher (81 tracked points)
2) BRISK detector with BRIEF matcher (74 tracked points)
3) BRISK detector with ORB matcher (75 tracked points)

***
My choices: BRISK corner detection with BRISK tracking (keypoint matcher). The consumed time seems feasible to me (less than 1ms, remark: I ran it on my PC locally). It is slower than most others, but the others
also result in no tracking at all.
***
