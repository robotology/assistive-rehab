# How to temporally align two signals

Some applications require signals to be temporally aligned.
For example, when analyzing and comparing the current and the template skeleton for producing a feedback, joints must be aligned.

Given two 1D signals, whose temporal samples are stored in vectors _v1_ and _v2_, you can use the following code snippet to get the aligned versions _w1_ and _w2_:  

```cpp
Dtw dtw(-1);
vector<double> w_v1,w_v2;
dtw.align(v1,v2,w_v1,w_v2);
double d = dtw.getDistance();
```

!!! tip
    In the example, no [adjustment window condition](https://robotology.github.io/assistive-rehab/doc/doxygen/doc/html/group__dtw.html) is applied. Therefore the search of the warping path is done along the whole distance matrix.
    To limit the search, you can create the obejct `dtw` by passing the desired window, e.g. `Dtw dtw(8)`.

For the multidimensional case, the code can be adapted as following:

```cpp
Dtw dtw(-1);
vector<vector<double>> w_v1,w_v2;
dtw.align(v1,v2,w_v1,w_v2);
double d = dtw.getDistance();
```

!!! note
    _v1_ and _v2_ are defined as `vector<vector>`.
