SFND 3D Object Tracking Project by Daniel Baudisch
==================================================


FP.1: Implement matchBoundingBoxes
----------------------------------

Function was implemented as suggested:
  * At the beginning the function creates a counter for each possible combination of "previous" boundingbox and "current"
    boundingbox.
  * For each match, the function takes the keypoint from the previous image and the keypoint from the current image. For
    the keypoint in the previous image, all boundingboxes for which the keypoint lies in are determined. Similar is done
    for the boundingboxes in the current image based on the keypoint in the current image. For each combination of the
    "previous" boundingboxes and the "current" boundingboxes the corresponding counter is incremented.
  * Finally, the mapping is created based on the counter array: for each "previous" boundingbox the function determines
    the "current" boundingbox with the most matches.


I also tried a different way: I assigned to each counter a value based on the overlap of two bounding boxes. Complexity
is O(#bounding boxes ^ 2) compared to the method above that has O(#matches * #bounding boxes).


Remark:
Something that I postponed way too long: update the visualization. During FP.3 and FP.4, I actually recognized that in a
single step that there are breaks just in the processing of one image. I did then a review and modifed the code in a way
that the processing of one image is done without a break. The drawing is done during the iteration os a single loop and
a "wait" is called only at the end. This showed that some steps were not executed due to some conditions. It helped a lot
in the debugging.



FP.2: TTC for Lidar
-------------------

The formula is straight forward and can be taken from previous lessons. However, to determine the right values for the
distance was quite a lot of experimenting with different statistical approaches (also to ensure that the computational
effort is kept low). In the end, a quite computing expensive operation is executed.

Tested approaches:
   * ***Select median:*** Sort points and select the median.
   * ***Compute average:*** Sort points, remove first and last _n_ points (assumption that there are a few outliners) and compute the average.
   * ***Compute average of closest points:*** Same as above, but use only closest _k_ points after removed the first _n_ points
   * ***Compute average until error falls under specified value:***
      1. Start with L = full list of point.
      1. Compute error for L ==> if lower then desired value, compute average of L.
      1. Compute error for L without first element.
      1. Compute error for L without last element.
      1. Remove the element causing the larger error.
      1. Go to step 2.

A more sophisticated algorithm should also consider the position in y direction. Example is already at hand: the car is
kind of "bend" and there is apparently one outliner in the middle, which seems to be okay-ish from its x-value.



FP.3 : Associate Keypoint Correspondences with Bounding Boxes
-------------------------------------------------------------

My first implementation seemed to have problems with outliers - especially with matches that have keypoints outside of the previous bounding box. Hence, the next step was to change the interface to also take the previous bounding box as argument into this function call.

Implementation:
   * Only consider the matches that have first keypoint in previous bounding box and second keypoint in current bounding box.
   * I had the filtering of matches based on Euclidean distance already implemented, but I did not see any changes on quality (after FP.4 was already implemented). So I commented the filtering out.



FP.4 : Compute Camera-based TTC
-------------------------------

I used the formula given in one of the previous lessons. The points is to determine h0 and h1 in the formula. To this end, h0 and h1 only need to be the distances between two matches (i.e. two pairs of tracked points). High-level idea is therefore:
   * Select a pair of matches (each match tracks a point)
   * Compute distance between points in previous frame (==> h0)
   * Compute distance between points in current frame (==> h1)
   * Compute TTC

Challenge was to select a good pair. Thereby, I considered following: Pairs of points that are close together will lead to a higher error. To get higher distances between a pair of tracked points, I sort the matches according to the Y value of the keypoints. I ran several tests and in many cases, that approach was already quite satisfying. However, some cases lead to quite high "noise". To improve in those cases, the algorithm uses several pairs, removes the ones that resulted in a NaN value and finally, I take the median out of the resulting list as result.



FP.5 : Performance Evaluation 1
-------------------------------

Observation:

Used methods to compute the TTC:
   * ttcMethod = TTCMedian (sort list, take median)
   * ttcMethod = TTCAverage10 (sort list, remove first and last ten elements, compute average)
   * ttcMethod = TTCAverage10_First10 (sort list, remove first ten elements, compute average of next ten elements)
   * ttcMethod = TTCAverageSmallestError

A run with following commands:
$./3D_object_tracking

&nbsp; &nbsp; &nbsp; L:Median &nbsp; &nbsp; &nbsp;  |&nbsp; &nbsp; &nbsp; L:Avg10 &nbsp; &nbsp; &nbsp;  |&nbsp; &nbsp; &nbsp; L:Avg10Cl10 &nbsp; &nbsp; &nbsp;  |&nbsp; &nbsp; &nbsp; L:AvgMinErr &nbsp; &nbsp; &nbsp;  |&nbsp; &nbsp; &nbsp; C:Median &nbsp; &nbsp; &nbsp;  |&nbsp; &nbsp; &nbsp; C:Avg10 &nbsp; &nbsp; &nbsp;  |&nbsp; &nbsp; &nbsp; C:Avg10Cl10 &nbsp; &nbsp; &nbsp;  |&nbsp; &nbsp; &nbsp; C:AvgMinErr &nbsp; &nbsp; &nbsp; 
---------------- |---------------- |---------------- |---------------- |---------------- |---------------- |---------------- |----------------
       12.515600 |       12.374812 |       13.806449 |       13.352985 |        1.256704 |        1.810627 |        0.071308 |        6.520647
       12.614245 |       13.216110 |       10.704909 |       12.152486 |       10.106185 |        8.006677 |        4.894691 |       10.106185
       14.091013 |       17.165788 |       14.970913 |       18.911302 |        4.384510 |        3.357245 |        3.197808 |        4.908068
       16.689386 |       14.900194 |       10.226255 |       14.598984 |        5.870845 |        3.546933 |               - |        6.654123
       15.908233 |       12.743410 |       12.312320 |       12.497964 |               - |        1.730855 |               - |        3.615809
       12.678716 |       13.544171 |        6.701063 |       14.802141 |        3.019400 |        3.377528 |        2.957307 |        3.019400
       11.984351 |       13.291290 |       16.467190 |       10.783008 |        2.920359 |        0.409687 |               - |       11.448349
       13.124118 |       13.963203 |       18.078185 |       15.423229 |        1.562084 |        0.924601 |        0.050147 |        3.874026
       13.024118 |       12.580945 |       11.277533 |       12.970127 |        3.282068 |        2.981885 |        0.076886 |        6.799235
       11.174641 |       12.341949 |       13.731587 |       12.806308 |        2.466865 |        2.652301 |               - |        2.565320
       12.808601 |       11.957508 |        9.441069 |       11.510780 |        6.580250 |        4.222187 |        4.457521 |        7.977774
        8.959780 |       10.237946 |        9.400632 |       10.223198 |        3.372656 |        3.611966 |        3.517671 |        9.044844
        9.964390 |        9.417084 |        8.273021 |        9.254543 |        4.990491 |        6.616758 |        3.112517 |       11.431390
        9.598630 |        9.427343 |       10.394741 |        9.474407 |        1.555110 |        1.313830 |               - |               -
        8.573525 |        8.440677 |        7.634313 |        8.321200 |        7.074812 |       22.950574 |        4.965627 |        7.730384
        9.516170 |        9.023848 |        8.700758 |        8.898673 |        3.472479 |        3.098455 |        2.954516 |        3.472479
        9.546581 |       11.250549 |        9.097104 |       11.030114 |        7.079633 |        3.135788 |        6.615466 |        8.864827
        8.398803 |        8.412031 |        8.980818 |        8.535568 |        0.427746 |        1.223593 |        0.271523 |        6.492742



   5.3708
   5.3905
***8.4837***
   5.2686
   3.8693
   5.5469
***10.009***
   7.4438
***26.612***
   6.1448
   8.6862
   6.8033
   5.9837
   6.0595
   6.9202
***9.8262***
   7.9798
   7.2532




The outliners are highlighted. The outlines clearly show a large (unexpected) increase of the TTC: the preceding vehicle gets
constantly closer every cycle (can be seen by pure "eye" measurement), hence the TTC must get closer to zero. For the third
method (TTCAverage10_First10), there is also one outlier to the other direction that might lead to a false positive due to the
strong drop, if it is not correctly handled in a fusion part afterwards.

I would go for the TTCMedian, since it seems to have pretty good values, despite of its outliers.



This exercise is about conducting tests with the final project code, especially with regard to the Lidar part. Look for several examples
where you have the impression that the Lidar-based TTC estimate is way off. Once you have found those, describe your observations and
provide a sound argumentation why you think this happened.

The task is complete once several examples (2-3) have been identified and described in detail. The assertion that the TTC is off should
be based on manually estimating the distance to the rear of the preceding vehicle from a top view perspective of the Lidar points.





FP.6 : Performance Evaluation 2
-------------------------------

This last exercise is about running the different detector / descriptor combinations and looking at the differences in TTC estimation. Find out which methods perform best and also include several examples where camera-based TTC estimation is way off. As with Lidar, describe your observations again and also look into potential reasons. This is the last task in the final project.

The task is complete once all detector / descriptor combinations implemented in previous chapters have been compared with regard to the TTC estimate on a frame-by-frame basis. To facilitate the comparison, a spreadsheet and graph should be used to represent the different TTCs.

We are super-excited to receive your submission as soon as possible.





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
