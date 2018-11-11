/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file nlp.cpp
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#include <algorithm>
#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>
#include <yarp/math/Math.h>
#include "nlp.h"

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;
using namespace assistive_rehab;


/****************************************************************/
class LimbOptimizerNLP : public Ipopt::TNLP
{
protected:
    const CamParamsHelper& camParams;
    const KeyPoint* k;
    const vector<double>& lengths;
    vector<pair<string,Vector>> result;

    /****************************************************************/
    Vector get2D(const Vector &x) const
    {
        Vector p(2,0.0);
        if (x[2]>0.0)
        {
            p[0]=camParams.get_focal()*(x[0]/x[2])+(camParams.get_width()-1)/2.0;
            p[1]=camParams.get_focal()*(x[1]/x[2])+(camParams.get_height()-1)/2.0;
        }
        return p;
    }

    /****************************************************************/
    Vector get3D(const Vector &p, const double d) const
    {
        Vector x(3,d);
        x[0]*=(p[0]-(camParams.get_width()-1)/2.0)/camParams.get_focal();
        x[1]*=(p[1]-(camParams.get_height()-1)/2.0)/camParams.get_focal();
        return x;
    }

    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m,
                      Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag,
                      IndexStyleEnum &index_style) override
    {
        n=0;
        for (auto c=k; c!=nullptr; c=c->getChild(0))
        {
            n++;
        }
        m=nnz_jac_g=nnz_h_lag=0;
        index_style=TNLP::C_STYLE;
        return true;
    }

    /****************************************************************/
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l,
                         Ipopt::Number *x_u, Ipopt::Index m,
                         Ipopt::Number *g_l, Ipopt::Number *g_u) override
    {
        Ipopt::Index i=0;
        for (auto c=k; c!=nullptr; c=c->getChild(0))
        {
            double d=(i==0?0.05:0.2);
            x_l[i]=std::max(c->getPoint()[2],0.01);
            x_u[i]=x_l[i]+d;
            i++;
        }
        return true;
    }

    /****************************************************************/
    bool get_starting_point(Ipopt::Index n, bool init_x,
                            Ipopt::Number *x, bool init_z,
                            Ipopt::Number *z_L, Ipopt::Number *z_U,
                            Ipopt::Index m, bool init_lambda,
                            Ipopt::Number *lambda) override
    {
        Ipopt::Index i=0;
        for (auto c=k; c!=nullptr; c=c->getChild(0))
        {
            x[i]=c->getPoint()[2];
            i++;
        }
        return true;
    }

    /****************************************************************/
    bool eval_f(Ipopt::Index n, const Ipopt::Number *x,
                bool new_x, Ipopt::Number &obj_value) override
    {
        obj_value=0.0; size_t i=0;
        for (auto c1=k; c1!=nullptr; c1=c1->getChild(0))
        {
            if (auto c2=c1->getChild(0))
            {
                Vector p1=get2D(c1->getPoint());
                Vector y1=get3D(p1,x[i]);

                Vector p2=get2D(c2->getPoint());
                Vector y2=get3D(p2,x[i+1]);

                Vector d1=y1-y2;
                double d2=dot(d1,d1)-lengths[i]*lengths[i];
                obj_value+=d2*d2;
                i++;
            }
        }
        return true;
    }

    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number *x,
                     bool new_x, Ipopt::Number *grad_f) override
    {
        for (Ipopt::Index i=0; i<n; i++)
        {
            grad_f[i]=0.0;
        }

        size_t i=0;
        for (auto c1=k; c1!=nullptr; c1=c1->getChild(0))
        {
            if (auto c2=c1->getChild(0))
            {
                Vector p1=get2D(c1->getPoint());
                Vector y1=get3D(p1,x[i]);
                Vector Dy1=get3D(p1,1.0);

                Vector p2=get2D(c2->getPoint());
                Vector y2=get3D(p2,x[i+1]);
                Vector Dy2=get3D(p2,1.0);

                Vector d1=y1-y2;
                double d2=dot(d1,d1)-lengths[i]*lengths[i];

                grad_f[i]+=4.0*d2*dot(d1,Dy1);
                grad_f[i+1]+=-4.0*d2*dot(d1,Dy2);
                i++;
            }
        }
        return true;
    }

    /****************************************************************/
    bool eval_g(Ipopt::Index n, const Ipopt::Number *x,
                bool new_x, Ipopt::Index m, Ipopt::Number *g) override
    {
        return true;
    }

    /****************************************************************/
    bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x,
                    bool new_x, Ipopt::Index m, Ipopt::Index nele_jac,
                    Ipopt::Index *iRow, Ipopt::Index *jCol,
                    Ipopt::Number *values) override
    {
        return true;
    }

    /****************************************************************/
    bool eval_h(Ipopt::Index n, const Ipopt::Number *x,
                bool new_x, Ipopt::Number obj_factor,
                Ipopt::Index m, const Ipopt::Number *lambda,
                bool new_lambda, Ipopt::Index nele_hess,
                Ipopt::Index *iRow, Ipopt::Index *jCol,
                Ipopt::Number *values) override
    {
        return true;
    }

    /****************************************************************/
    void finalize_solution(Ipopt::SolverReturn status,
                           Ipopt::Index n, const Ipopt::Number *x,
                           const Ipopt::Number *z_L,
                           const Ipopt::Number *z_U,
                           Ipopt::Index m, const Ipopt::Number *g,
                           const Ipopt::Number *lambda,
                           Ipopt::Number obj_value,
                           const Ipopt::IpoptData *ip_data,
                           Ipopt::IpoptCalculatedQuantities *ip_cq) override
    {
        Ipopt::Index i=0;
        for (auto c=k; c!=nullptr; c=c->getChild(0))
        {
            Vector v=get2D(c->getPoint());
            result.push_back(make_pair(c->getTag(),get3D(v,x[i++])));
        }
    }

public:
    /****************************************************************/
    LimbOptimizerNLP(const CamParamsHelper &camParams_, const KeyPoint* k_,
                     const vector<double>& lengths_) :
                     camParams(camParams_), k(k_), lengths(lengths_) { }

    /****************************************************************/
    vector<pair<string,Vector>> get_result() const
    {
        return result;
    }
};


/****************************************************************/
vector<pair<string,Vector>> LimbOptimizer::optimize(const CamParamsHelper &camParams,
                                                    const KeyPoint* k,
                                                    const vector<double>& lengths)
{
    
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
    app->Options()->SetNumericValue("tol",0.0001);
    app->Options()->SetIntegerValue("acceptable_iter",0);
    app->Options()->SetStringValue("mu_strategy","adaptive");
    app->Options()->SetStringValue("hessian_approximation","limited-memory");
    app->Options()->SetIntegerValue("max_iter",100);
    app->Options()->SetNumericValue("max_cpu_time",0.05);
    app->Options()->SetIntegerValue("print_level",0);
    app->Initialize();

    Ipopt::SmartPtr<LimbOptimizerNLP> nlp=new LimbOptimizerNLP(camParams,k,lengths);
    Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(nlp));
    switch (status)
    {
        case Ipopt::Solve_Succeeded:
        case Ipopt::Solved_To_Acceptable_Level:
        case Ipopt::Feasible_Point_Found:
        {
            return nlp->get_result();
        }
        default:
        {
            return vector<pair<string,Vector>>();
        } 
    }
}

