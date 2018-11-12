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
    const double center_depth;
    const KeyPoint* k;
    const vector<double>& lengths;
    vector<pair<string,Vector>> result;

    vector<Vector> p1,y1,Dy1;
    vector<Vector> p2,y2,Dy2;
    vector<Vector> d1;
    vector<double> d2;

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
    void compute_quantities(const Ipopt::Number *x)
    {
        size_t i=0;
        for (auto c1=k; c1!=nullptr; c1=c1->getChild(0))
        {
            if (auto c2=c1->getChild(0))
            {
                p1[i]=get2D(c1->getPoint());
                y1[i]=get3D(p1[i],x[i]);
                Dy1[i]=get3D(p1[i],1.0);

                p2[i]=get2D(c2->getPoint());
                y2[i]=get3D(p2[i],x[i+1]);
                Dy2[i]=get3D(p2[i],1.0);

                d1[i]=y1[i]-y2[i];
                d2[i]=dot(d1[i],d1[i])-lengths[i]*lengths[i];
                i++;
            }
            else
            {
                break;
            }
        }
    }

    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m,
                      Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag,
                      IndexStyleEnum &index_style) override
    {
        n=(Ipopt::Index)d2.size()+1;
        m=nnz_jac_g=nnz_h_lag=0;
        index_style=TNLP::C_STYLE;
        return true;
    }

    /****************************************************************/
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l,
                         Ipopt::Number *x_u, Ipopt::Index m,
                         Ipopt::Number *g_l, Ipopt::Number *g_u) override
    {
        // anchor the first keypoint to center
        x_l[0]=std::max(center_depth,0.01);
        x_u[0]=x_l[0]+0.01;

        Ipopt::Index i=1;
        for (auto c=k->getChild(0); c!=nullptr; c=c->getChild(0))
        {
            x_l[i]=std::max(c->getPoint()[2],0.01);
            x_u[i]=x_l[i]+0.3;
            i++;
        }

        // anchor the last keypoint to the its depth
        x_u[n-1]=x_l[n-1]+0.01;
        return true;
    }

    /****************************************************************/
    bool get_starting_point(Ipopt::Index n, bool init_x,
                            Ipopt::Number *x, bool init_z,
                            Ipopt::Number *z_L, Ipopt::Number *z_U,
                            Ipopt::Index m, bool init_lambda,
                            Ipopt::Number *lambda) override
    {
        x[0]=center_depth;

        Ipopt::Index i=1;
        for (auto c=k->getChild(0); c!=nullptr; c=c->getChild(0))
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
        if (new_x)
        {
            compute_quantities(x);
        }

        obj_value=0.0;
        for (size_t i=0; i<d2.size(); i++)
        {
            obj_value+=d2[i]*d2[i];
        }
        return true;
    }

    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number *x,
                     bool new_x, Ipopt::Number *grad_f) override
    {
        if (new_x)
        {
            compute_quantities(x);
        }

        for (Ipopt::Index i=0; i<n; i++)
        {
            grad_f[i]=0.0;
        }

        for (size_t i=0; i<d2.size(); i++)
        {
            grad_f[i]+=4.0*d2[i]*dot(d1[i],Dy1[i]);
            grad_f[i+1]-=4.0*d2[i]*dot(d1[i],Dy2[i]);
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
            result.push_back(make_pair(c->getTag(),get3D(v,x[i])));
            i++;
        }
    }

public:
    /****************************************************************/
    LimbOptimizerNLP(const CamParamsHelper &camParams_, const double center_depth_,
                     const KeyPoint* k_, const vector<double>& lengths_) :
                     camParams(camParams_), center_depth(center_depth_),
                     k(k_), lengths(lengths_)
    {
        size_t i=0;
        for (auto c1=k; c1!=nullptr; c1=c1->getChild(0))
        {
            i++;
        }
        i--;
        p1=vector<Vector>(i); y1=vector<Vector>(i); Dy1=vector<Vector>(i);
        p2=vector<Vector>(i); y2=vector<Vector>(i); Dy2=vector<Vector>(i);
        d1=vector<Vector>(i); d2=vector<double>(i);
    }

    /****************************************************************/
    vector<pair<string,Vector>> get_result() const
    {
        return result;
    }
};


/****************************************************************/
vector<pair<string,Vector>> LimbOptimizer::optimize(const CamParamsHelper &camParams,
                                                    const double center_depth,
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

    Ipopt::SmartPtr<LimbOptimizerNLP> nlp=new LimbOptimizerNLP(camParams,center_depth,k,lengths);
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

