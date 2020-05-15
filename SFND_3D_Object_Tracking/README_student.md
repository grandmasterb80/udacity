SFND 3D Object Tracking Project by Daniel Baudisch (Resubmission)
==================================================================


FP.1: Implement matchBoundingBoxes
----------------------------------

Function was implemented as suggested:
  * At the beginning the function creates a counter for each possible combination of "previous" boundingbox and "current" boundingbox.
  * For each match, the function takes the keypoint from the previous image and the keypoint from the current image. For the keypoint in the previous image, all boundingboxes for which the keypoint lies in are determined. Similar is done for the boundingboxes in the current image based on the keypoint in the current image. For each combination of the "previous" boundingboxes and the "current" boundingboxes the corresponding counter is incremented.
  * Finally, the mapping is created based on the counter array: for each "previous" boundingbox the function determines the "current" boundingbox with the most matches.


I also tried a different way: I assigned to each counter a value based on the overlap of two bounding boxes. Complexity is O(#bounding boxes ^ 2) compared to the method above that has O(#matches * #bounding boxes).


Remark:
Something that I postponed way too long: update the visualization. During FP.3 and FP.4, I actually recognized that in a single step that there are breaks just in the processing of one image. I did then a review and modifed the code in a way that the processing of one image is done without a break. The drawing is done during the iteration os a single loop and a "wait" is called only at the end. This showed that some steps were not executed due to some conditions. It helped a lot in the debugging.



FP.2: TTC for Lidar
-------------------

The formula is straight forward and can be taken from previous lessons. However, to determine the right values for the distance was quite a lot of experimenting with different statistical approaches (also to ensure that the computational effort is kept low). In the end, a quite computing expensive operation is executed. I would compare it to median in the next step when I had more data. In the end, the question is also how much we can get by applying Kalman filtering.

The idea to get the distance is to sort the array with keypoint, such that it is easier to remove outliners based on their Z-value. A more sophisticated algorithm should also consider the position in y direction. Example is already at hand: the car is kind of "bend" and there is apparently one outliner in the middle, which seems to be okay-ish from its x-value.



FP.3 : Associate Keypoint Correspondences with Bounding Boxes
-------------------------------------------------------------

My first implementation seemed to have problems with outliers - especially with matches that have keypoints outside of the previous bounding box. Hence, the next step was to change the interface to also take the previous bounding box as argument into this function call.

Implementation:
  * Only consider the matches that have first keypoint in previous bounding box and second keypoint in current bounding box.
  * I had the filtering of matches based on Euclidean distance already implemented, but I did not see any changes on quality (after FP.4 was already implemented). So I commented the filtering out.



FP.4 : Compute Camera-based TTC
-------------------------------






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
