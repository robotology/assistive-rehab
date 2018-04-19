/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file nlp.h
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#ifndef NLP_H
#define NLP_H

#include <vector>
#include <utility>
#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include "AssistiveRehab/skeleton.h"


/****************************************************************/
class LimbOptimizer : public Ipopt::TNLP
{
protected:
    const assistive_rehab::KeyPoint* k;
    std::vector<double> lengths;

    Ipopt::Index num_var;
    std::vector<std::pair<std::string,yarp::sig::Vector>> result;

    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style) override;

    /****************************************************************/
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                         Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u) override;

    /****************************************************************/
    bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                            bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                            Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda) override;

    /****************************************************************/
    bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number &obj_value) override;

    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                     Ipopt::Number *grad_f) override;

    /****************************************************************/
    bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Index m, Ipopt::Number *g) override;

    /****************************************************************/
    bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                    Ipopt::Index *jCol, Ipopt::Number *values) override;

    /****************************************************************/
    bool eval_h(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number *lambda,
                bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index *iRow,
                Ipopt::Index *jCol, Ipopt::Number *values) override;

    /****************************************************************/
    void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                           const Ipopt::Number *x, const Ipopt::Number *z_L,
                           const Ipopt::Number *z_U, Ipopt::Index m,
                           const Ipopt::Number *g, const Ipopt::Number *lambda,
                           Ipopt::Number obj_value, const Ipopt::IpoptData *ip_data,
                           Ipopt::IpoptCalculatedQuantities *ip_cq) override;

public:
    /****************************************************************/
    LimbOptimizer(const assistive_rehab::KeyPoint* k_,
                  std::vector<double> lengths_);

    /****************************************************************/
    std::vector<std::pair<std::string,yarp::sig::Vector>> get_result() const { return result; }
};

#endif

