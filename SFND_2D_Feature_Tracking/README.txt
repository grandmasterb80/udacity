SFND_2D_Feature_Tracking Project by Daniel Baudisch
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
----------------------------------------------------

Instead of executing all the different combinations manually, I added a loop to iterate through all the iterations. For each iterations, 






######################################
to be deleted:

Heading
=======

Sub-heading
-----------

Paragraphs are separated
by a blank line.

Two spaces at the end of a line  
produces a line break.

Text attributes _italic_, 
**bold**, `monospace`.

Horizontal rule:

---

Strikethrough:
~~strikethrough~~

Bullet list:

  * apples
  * oranges
  * pears

Numbered list:

  1. lather
  2. rinse
  3. repeat

An [example](http://example.com).

![Image](Icon-pictures.png "icon")

> Markdown uses email-style > characters for blockquoting.

Inline <abbr title="Hypertext Markup Language">HTML</abbr> is supported.
