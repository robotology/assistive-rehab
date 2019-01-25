# How to temporally align two signals

Some applications require signals to be temporally aligned.
For example, when analyzing and comparing the current and the template skeleton for producing a feedback, joints must be aligned.

Given two 1D signals, whose temporal samples are stored in vectors _v1_ and _v2_, you can use the following code snippet to get the aligned versions _w1_ and _w2_:  

```cpp
assistive_rehab::Dtw dtw(-1);
std::vector<double> w_v1,w_v2;
dtw.align(v1,v2,w_v1,w_v2);
double d = dtw.getDistance();
```

!!! tip
    In the example, no [adjustment window condition](https://robotology.github.io/assistive-rehab/doc/doxygen/doc/html/group__dtw.html) is applied. Therefore the search of the warping path is done along the whole distance matrix.
    To limit the search, you can create the obejct `dtw` by passing the desired window, e.g. `assistive_rehab::Dtw dtw(8)`.

The following images show two signals before and after the application of DTW:

!!! example "Before DTW"
    ![before-dtw](https://user-images.githubusercontent.com/9716288/51757022-a70e7e80-20c2-11e9-9fc2-faee13550209.png)

!!! example "After DTW"
    ![after-dtw](https://user-images.githubusercontent.com/9716288/51757033-ad9cf600-20c2-11e9-8946-c31e25629a60.png)  


For the multidimensional case, the code can be adapted as following:

```cpp
assistive_rehab::Dtw dtw(-1);
std::vector<std::vector<double>> w_v1,w_v2;
dtw.align(v1,v2,w_v1,w_v2);
double d = dtw.getDistance();
```

!!! note
    _v1_ and _v2_ are defined as `std::vector<std::vector>`.
