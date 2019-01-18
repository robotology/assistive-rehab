/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * \defgroup dtw dtw
 *
 * Class for temporal alignment based on Dynamic Time Warping (DTW).
 *
 * \section intro_sec Description
 *
 * The class Dtw can be used for aligning two temporal sequences using the Dynamic Time Warping (DTW).
 * DTW calculates the optimal path which minimizes the distance between the two signals to be aligned.
 * Given two signals \f$s\f$ and \f$t\f$, with \f$n_s\f$ and \f$n_t\f$ samples respectively,
 * DTW constructs the \f$n_s x n_t\f$ distance matrix \f$D\f$,
 * where \f$D_{(i,j)} = d(s_i,t_j) + \min{(d(i-1,j-1),d(i-1,j),d(i,j-1))}\f$,
 * and \f$d(i,j) = \sqrt{(s_i-t_i)^2}\f$.
 * A warping path has to satisfy the following conditions:
 * - monotonic condition: the path monotonically increases;
 * - continuity condition: the path advances one step at a time;
 * - boundary condition: the path starts at bottom left and ends at top right.
 * The optimal warping path \f$w*\f$ with length \f$k\f$ is the one that minimizes the total distance
 * among all possible warping paths:
 * \f[d_{DTW} = \min_{i=1}^k D(w_i)\f]
 * The DTW distance is the total distance of the optimal warping path.
 *
 * DTW can be also applied to multidimensional temporal sequences, by applying the same procedure independently
 * to the corresponding components of the two signals. The DTW distance is the sum of the DTW distances of
 * each component.
 *
 * Additional constraints can be used to restrict the space of search of the warping path.
 * The library includes the adjustment window condition, which enforces the search of the warping path inside
 * a window around the distance matrix diagonal.
 *
 * \section code_example_sec Example
 *
 * Given two vectors v1,v2, the following piece of code can be used to align them and get the DTW distance:
 *
 * \code
 * Dtw dtw(-1);
 * vector<double> w_v1,w_v2; //aligned signals
 * dtw.align(v1,v2,w_v1,w_v2);
 * double d = dtw.getDistance();
 * \endcode
 *
 * For the multidimensional case, given two signals with \f$n\f$ samples over time and \f$m\f$ components,
 * v1 and v2 have to be defined as \f$n\f$ vector of \f$m\f$ vectors.
 * The following piece of code can be used to align them and get the DTW distance:
 *
 * \code
 * Dtw dtw(-1);
 * vector<vector<double>> w_v1,w_v2; //aligned signals are now defined as vector of vectors
 * dtw.align(v1,v2,w_v1,w_v2);
 * double d = dtw.getDistance();
 * \endcode
 *
 * \author Valentina Vasco <valentina.vasco@iit.it>
 */

#ifndef ASSISTIVE_REHAB_DTW_H
#define ASSISTIVE_REHAB_DTW_H

#include <vector>
#include <yarp/sig/Matrix.h>

namespace assistive_rehab
{

/**
* Class for DTW (Dynamic Time Warping).
*/
class Dtw
{
protected:
    int win;   /**< window length where the warping path is searched */
    double d;   /**< dtw distance */

    /**
    * Set the elements of the distance matrix to -1, except the (0,0) element set at 0.
    * @param ns length of input vector s.
    * @param nt length of input vector t.
    * @return a ns x nt distance matrix.
    */
    yarp::sig::Matrix initialize(const int ns, const int nt) const;

    /**
    * Retrieve the optimal warping path.
    * @param distMat distance matrix.
    * @param ws vector containing the output warping path of s.
    * @param wt vector containing the output warping path of t.
    */
    void getWarpingPath(const yarp::sig::Matrix &distMat, std::vector<int> &ws, std::vector<int> &wt) const;

    /**
    * Compute the DTW distance.
    * @param s vector containing input signal s.
    * @param t vector containing input signal t.
    * @param distMat matrix containing the output distance matrix.
    * @return DTW distance.
    */
    double computeDistance(const std::vector<double> &s, const std::vector<double> &t,
                           yarp::sig::Matrix &distMat) const;

public:

    /**
    * Default constructor.
    */
    Dtw();

    /**
    * Overloaded constructor.
    * @param win_ window length where we look for the warping path.
    * If win_ = -1, the search is carried out on the whole distance matrix.
    */
    Dtw(const int &win_);

    /**
    * Align two 1D temporal sequences s,t.
    * @param s vector containing the input signal s.
    * @param t vector containing the input signal t.
    * @param ws vector containing the aligned output s.
    * @param wt vector containing the aligned output t.
    */
    void align(const std::vector<double> &s, const std::vector<double> &t,
               std::vector<double> &ws, std::vector<double> &wt);

    /**
    * Align two ND temporal sequences s,t.
    * @param s vector of vectors containing the N components over time of the input signal s.
    * @param t vector of vectors containing the N components over time of the input signal t.
    * @param ws vector of vectors containing the N components over time of the aligned output s.
    * @param wt vector of vectors containing the N components over time of the aligned output t.
    */
    void align(const std::vector<std::vector<double>> &s, const std::vector<std::vector<double>> &t,
               std::vector<std::vector<double> > &ws, std::vector<std::vector<double> > &wt);

    /**
    * Retrieve the optimal distance.
    * @return optimal distance.
    */
    double getDistance() const { return d; }

    /**
    * Virtual destructor.
    */
    virtual ~Dtw() { }
};

}

#endif
