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
