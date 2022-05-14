
#ifndef REEDSSHEPP_CPP
#define REEDSSHEPP_CPP

#include "reedsshepp.h"
#define _USE_MATH_DEFINES // for C++
#include <math.h>
#include <assert.h>
#include "algorithm.h"
#include <boost/heap/binomial_heap.hpp>
#include <typeinfo>

#define EPSILON (10e-10)

// not used for RS curve calculation
#define LSL (0)
#define LSR (1)
#define RSL (2)
#define RSR (3)
#define RLR (4)
#define LRL (5)

// The four segment types a path can be made up of
#define L_SEG (0)
#define S_SEG (1)
#define R_SEG (2)
#define NO_SEG (3)

namespace HybridAStar
{

// The segment types for each of the Path types
const int DIRDATA[][5] = {
    {L_SEG, R_SEG, L_SEG, NO_SEG, NO_SEG},
    {R_SEG, L_SEG, R_SEG, NO_SEG, NO_SEG},
    {L_SEG, R_SEG, L_SEG, R_SEG, NO_SEG},
    {R_SEG, L_SEG, R_SEG, L_SEG, NO_SEG},
    {L_SEG, R_SEG, S_SEG, L_SEG, NO_SEG},
    {R_SEG, L_SEG, S_SEG, R_SEG, NO_SEG},
    {L_SEG, S_SEG, R_SEG, L_SEG, NO_SEG},
    {R_SEG, S_SEG, L_SEG, R_SEG, NO_SEG},
    {L_SEG, R_SEG, S_SEG, R_SEG, NO_SEG},
    {R_SEG, L_SEG, S_SEG, L_SEG, NO_SEG},
    {R_SEG, S_SEG, R_SEG, L_SEG, NO_SEG},
    {L_SEG, S_SEG, L_SEG, R_SEG, NO_SEG},
    {L_SEG, S_SEG, R_SEG, NO_SEG, NO_SEG},
    {R_SEG, S_SEG, L_SEG, NO_SEG, NO_SEG},
    {L_SEG, S_SEG, L_SEG, NO_SEG, NO_SEG},
    {R_SEG, S_SEG, R_SEG, NO_SEG, NO_SEG},
    {L_SEG, R_SEG, S_SEG, L_SEG, R_SEG},
    {R_SEG, L_SEG, S_SEG, R_SEG, L_SEG}};

ReedsSheppWord reedsshepp_words[] = {
    reedsshepp_LSL,
    reedsshepp_LSR,
    reedsshepp_RSL,
    reedsshepp_RSR,
    reedsshepp_RLR,
    reedsshepp_LRL,
};

#define UNPACK_INPUTS(alpha, beta) \
    double sa = sin(alpha);        \
    double sb = sin(beta);         \
    double ca = cos(alpha);        \
    double cb = cos(beta);         \
    double c_ab = cos(alpha - beta);

#define PACK_OUTPUTS(outputs) \
    outputs[0] = t;           \
    outputs[1] = p;           \
    outputs[2] = q;

/**
 * Floating point modulus suitable for rings
 *
 * fmod doesn't behave correctly for angular quantities, this function does
 */

double fmodr(double x, double y)
{
    return x - y * floor(x / y);
}

double mod2pi(double theta)
{
    return fmodr(theta, 2 * M_PI);
}

int reedsshepp_init_normalised(double alpha, double beta, double d, ReedsSheppPath *path)
{
    double best_cost = INFINITY;
    int best_word;
    int i;

    best_word = -1;
    for (i = 0; i < 6; i++)
    {
        double params[3];
        int err = reedsshepp_words[i](alpha, beta, d, params);
        if (err == EDUBOK)
        {
            double cost = params[0] + params[1] + params[2];
            if (cost < best_cost)
            {
                best_word = i;
                best_cost = cost;
                path->param[0] = params[0];
                path->param[1] = params[1];
                path->param[2] = params[2];
                path->type = i;
            }
        }
    }

    if (best_word == -1)
    {
        return EDUBNOPATH;
    }
    path->type = best_word;
    return EDUBOK;
}

int reedsshepp_init(double q0[3], double q1[3], double rho, ReedsSheppPath *path, int goalposleft)
{
    int i;
    // double dx = q1[0] - q0[0];
    // double dy = q1[1] - q0[1];
    // double D = sqrt(dx * dx + dy * dy);
    // double d = D / rho;
    // if (rho <= 0.)
    // {
    //     return EDUBBADRHO;
    // }
    // double theta = mod2pi(atan2(dy, dx));
    // double alpha = mod2pi(q0[2] - theta);
    // double beta = mod2pi(q1[2] - theta);
    for (i = 0; i < 3; i++)
    {
        path->qi[i] = q0[i];
    }
    path->rho = rho;

    /* Combine init_normalised part with init */
    int best_word = -1;

    float reedsSheppLength = 0.f;
    ompl::base::ReedsSheppStateSpace::ReedsSheppPath path_ompl;
    ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
    State *rsStart = (State *)reedsSheppPath.allocState();
    State *rsEnd = (State *)reedsSheppPath.allocState();
    rsStart->setXY(q0[0], q0[1]);
    rsStart->setYaw(q0[2]);
    rsEnd->setXY(q1[0], q1[1]);
    rsEnd->setYaw(q1[2]);
    reedsSheppLength = reedsSheppPath.distance(rsStart, rsEnd);
    //cout<<"RS length in ompl: "<<reedsSheppLength<<endl;
    path_ompl = reedsSheppPath.reedsShepp(rsStart, rsEnd);
    // cout << "goalposleft:  "<< goalposleft<<endl;
    // 遍历判断路径类型
    for (int i = 0; i < 18; i++)
    {
        if(goalposleft == 1)
        {
            // left
            // reject left traj when forward and right when reverse
            if(DIRDATA[i][0] == L_SEG && path_ompl.length_[0] > 0){continue;}
            if(DIRDATA[i][0] == R_SEG && path_ompl.length_[0] < 0){continue;}
            bool flag = false;
            for( int j = 0; j < 5; j++)
            {
                if(DIRDATA[i][j] == L_SEG && path_ompl.length_[j] > 1) {flag = true;}
                if(DIRDATA[i][j] == R_SEG && path_ompl.length_[j] < -0.5) {flag = true;}
                // aviod last path forward
                if(j < 4)
                {
                    if(path_ompl.length_[j] > 0 && path_ompl.length_[j+1] == 0) {flag = true;}
                }
                else if (path_ompl.length_[j] > 0){flag = true;}
            }
            if(flag) {continue;}
        }
        else if(goalposleft == 0)
        {
            // right
            // reject right traj when forward and left when reverse
            if(DIRDATA[i][0] == L_SEG && path_ompl.length_[0] < 0){continue;}
            if(DIRDATA[i][0] == R_SEG && path_ompl.length_[0] > 0){continue;}
            bool flag = false;
            for( int j = 0; j < 5; j++)
            {
                if(DIRDATA[i][j] == R_SEG && path_ompl.length_[j] > 1) {flag = true;}
                if(DIRDATA[i][j] == L_SEG && path_ompl.length_[j] < -0.5) {flag = true;}
                // aviod last path forward
                if(j < 4)
                {
                    if(path_ompl.length_[j] > 0 && path_ompl.length_[j+1] == 0) {flag = true;}
                }
                else if (path_ompl.length_[j] > 0){flag = true;}
            }
            if(flag) {continue;}
        }
        if (path_ompl.type_ == ompl::base::ReedsSheppStateSpace::reedsSheppPathType[i])
        {
            best_word = i;
            //cout<<"I found it, and it is"<<i<<"and"<<ompl::base::ReedsSheppStateSpace::reedsSheppPathType[i]<<endl;
        }
    }
    // path->param[0] = path_ompl.length_[0] / rho;
    // path->param[1] = path_ompl.length_[1] / rho;
    // path->param[2] = path_ompl.length_[2] / rho;
    // path->param[3] = path_ompl.length_[3] / rho;
    // path->param[4] = path_ompl.length_[4] / rho;
    path->param[0] = path_ompl.length_[0];
    path->param[1] = path_ompl.length_[1];
    path->param[2] = path_ompl.length_[2];
    path->param[3] = path_ompl.length_[3];
    path->param[4] = path_ompl.length_[4];
    //cout<<"0: "<<path->param[0]<<"\t"<<path_ompl.length_[0]<<endl;
    /* length can be either positive or negative and means the original lenth value */
    path->type = best_word;
    if (best_word == -1)
    {
        return EDUBNOPATH;
    }
    return EDUBOK;

    //return reedsshepp_init_normalised( alpha, beta, d, path );
}

int reedsshepp_LSL(double alpha, double beta, double d, double *outputs)
{
    UNPACK_INPUTS(alpha, beta);
    double tmp0 = d + sa - sb;
    double p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sa - sb));
    if (p_squared < 0)
    {
        return EDUBNOPATH;
    }
    double tmp1 = atan2((cb - ca), tmp0);
    double t = mod2pi(-alpha + tmp1);
    double p = sqrt(p_squared);
    double q = mod2pi(beta - tmp1);
    PACK_OUTPUTS(outputs);
    return EDUBOK;
}

int reedsshepp_RSR(double alpha, double beta, double d, double *outputs)
{
    UNPACK_INPUTS(alpha, beta);
    double tmp0 = d - sa + sb;
    double p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sb - sa));
    if (p_squared < 0)
    {
        return EDUBNOPATH;
    }
    double tmp1 = atan2((ca - cb), tmp0);
    double t = mod2pi(alpha - tmp1);
    double p = sqrt(p_squared);
    double q = mod2pi(-beta + tmp1);
    PACK_OUTPUTS(outputs);
    return EDUBOK;
}

int reedsshepp_LSR(double alpha, double beta, double d, double *outputs)
{
    UNPACK_INPUTS(alpha, beta);
    double p_squared = -2 + (d * d) + (2 * c_ab) + (2 * d * (sa + sb));
    if (p_squared < 0)
    {
        return EDUBNOPATH;
    }
    double p = sqrt(p_squared);
    double tmp2 = atan2((-ca - cb), (d + sa + sb)) - atan2(-2.0, p);
    double t = mod2pi(-alpha + tmp2);
    double q = mod2pi(-mod2pi(beta) + tmp2);
    PACK_OUTPUTS(outputs);
    return EDUBOK;
}

int reedsshepp_RSL(double alpha, double beta, double d, double *outputs)
{
    UNPACK_INPUTS(alpha, beta);
    double p_squared = (d * d) - 2 + (2 * c_ab) - (2 * d * (sa + sb));
    if (p_squared < 0)
    {
        return EDUBNOPATH;
    }
    double p = sqrt(p_squared);
    double tmp2 = atan2((ca + cb), (d - sa - sb)) - atan2(2.0, p);
    double t = mod2pi(alpha - tmp2);
    double q = mod2pi(beta - tmp2);
    PACK_OUTPUTS(outputs);
    return EDUBOK;
}

int reedsshepp_RLR(double alpha, double beta, double d, double *outputs)
{
    UNPACK_INPUTS(alpha, beta);
    double tmp_rlr = (6. - d * d + 2 * c_ab + 2 * d * (sa - sb)) / 8.;
    if (fabs(tmp_rlr) > 1)
    {
        return EDUBNOPATH;
    }
    double p = mod2pi(2 * M_PI - acos(tmp_rlr));
    double t = mod2pi(alpha - atan2(ca - cb, d - sa + sb) + mod2pi(p / 2.));
    double q = mod2pi(alpha - beta - t + mod2pi(p));
    PACK_OUTPUTS(outputs);
    return EDUBOK;
}

int reedsshepp_LRL(double alpha, double beta, double d, double *outputs)
{
    UNPACK_INPUTS(alpha, beta);
    double tmp_lrl = (6. - d * d + 2 * c_ab + 2 * d * (-sa + sb)) / 8.;
    if (fabs(tmp_lrl) > 1)
    {
        return EDUBNOPATH;
    }
    double p = mod2pi(2 * M_PI - acos(tmp_lrl));
    double t = mod2pi(-alpha - atan2(ca - cb, d + sa - sb) + p / 2.);
    double q = mod2pi(mod2pi(beta) - alpha - t + mod2pi(p));
    PACK_OUTPUTS(outputs);
    return EDUBOK;
}

double reedsshepp_path_length(ReedsSheppPath *path)
{
    double length = 0.;
    length += fabs(path->param[0]);
    length += fabs(path->param[1]);
    length += fabs(path->param[2]);
    length += fabs(path->param[3]);
    length += fabs(path->param[4]);
    length = length * path->rho;
    //cout<<"length in func: "<<length<<"rho: "<<path->rho<<endl;
    return length;
}

int reedsshepp_path_type(ReedsSheppPath *path)
{
    return path->type;
}

void reedsshepp_segment(double t, double qi[3], double qt[3], int type)
{
    assert(type == L_SEG || type == S_SEG || type == R_SEG || type == NO_SEG);

    // forward condition and backward condition can be unified in this form?
    if (type == L_SEG)
    {
        qt[0] = qi[0] + sin(qi[2] + t) - sin(qi[2]);
        qt[1] = qi[1] - cos(qi[2] + t) + cos(qi[2]);
        qt[2] = qi[2] + t;
    }
    else if (type == R_SEG)
    {
        qt[0] = qi[0] - sin(qi[2] - t) + sin(qi[2]);
        qt[1] = qi[1] + cos(qi[2] - t) - cos(qi[2]);
        qt[2] = qi[2] - t;
    }
    else if (type == S_SEG)
    {
        qt[0] = qi[0] + cos(qi[2]) * t;
        qt[1] = qi[1] + sin(qi[2]) * t;
        qt[2] = qi[2];
    }
    else if (type == NO_SEG)
    {
        qt[0] = qi[0];
        qt[1] = qi[1];
        qt[2] = qi[2];
    }

    /*
    cout<<"my type is:  "<< type <<" and my length is t: "<<t<<endl;
    cout<<"qi value: "<<qi[0]<<"   "<<qi[1]<<"   "<<qi[2]<<endl;
    cout<<"qt value: "<<qt[0]<<"   "<<qt[1]<<"   "<<qt[2]<<endl;
    */

    /* backward condition
    else if( type == L_SEG && t < 0 ) {
        qt[0] = qi[0] + sin(qi[2]+t) - sin(qi[2]);
        qt[1] = qi[1] + cos(qi[2]+t) - cos(qi[2]);
        qt[2] = qi[2] - t;
    }
    else if( type == R_SEG && t < 0) {
        qt[0] = qi[0] - sin(qi[2]-t) + sin(qi[2]);
        qt[1] = qi[1] - cos(qi[2]-t) + cos(qi[2]);
        qt[2] = qi[2] + t;
    }
    else if( type == S_SEG && t < 0 ) {
        qt[0] = qi[0] + cos(qi[2]) * t;
        qt[1] = qi[1] + sin(qi[2]) * t;
        qt[2] = qi[2];
    }
    */
}

int reedsshepp_path_sample(ReedsSheppPath *path, double t, double q[3], int prim_val[1])
{
    if (t < 0 || t >= reedsshepp_path_length(path))
    {
        // error, parameter out of bounds
        return EDUBPARAM;
    }

    // tprime is the normalised variant of the parameter t
    double tprime = t / path->rho;

    // In order to take rho != 1 into account this function needs to be more complex
    // than it would be otherwise. The transformation is done in five stages.
    //
    // 1. translate the components of the initial configuration to the origin
    // 2. generate the target configuration
    // 3. transform the target configuration
    //      scale the target configuration
    //      translate the target configration back to the original starting point
    //      normalise the target configurations angular component

    // The translated initial configuration
    double qi[3] = {0, 0, path->qi[2]};

    // Generate the target configuration
    const int *types = DIRDATA[path->type];
    double p1 = path->param[0];
    double p2 = path->param[1];
    double p3 = path->param[2];
    double p4 = path->param[3];
    double p5 = path->param[4];

    double L1 = fabs(p1);
    double L2 = fabs(p2);
    double L3 = fabs(p3);
    double L4 = fabs(p4);
    double L5 = fabs(p5);

    // int sign1 = round(p1 / L1);
    // int sign2 = round(p2 / L2);
    // int sign3 = round(p3 / L3);
    // int sign4 = round(p4 / L4);
    // int sign5 = round(p5 / L5);
    // if (L4 < 1e-15)
    // {
    //     sign4 = 0;
    // }
    // if (L5 < 1e-15)
    // {
    //     sign5 = 0;
    // }

    // 求p1-p5的符号
    
    int sign1 = getSign(p1);
    int sign2 = getSign(p2);
    int sign3 = getSign(p3);
    int sign4 = getSign(p4);
    int sign5 = getSign(p5);

    double q1[3]; // end-of segment 1
    double q2[3]; // end-of segment 2
    double q3[3]; // end-of segment 3
    double q4[3]; // end-of segment 4

    reedsshepp_segment(p1, qi, q1, types[0]);
    reedsshepp_segment(p2, q1, q2, types[1]);
    reedsshepp_segment(p3, q2, q3, types[2]);
    reedsshepp_segment(p4, q3, q4, types[3]);
    //cout<<"sign"<<sign1<<"  "<<sign2<<"  "<<sign3<<"  "<<sign4<<"  "<<sign5<<endl;
    if (tprime < L1)
    {
        reedsshepp_segment(sign1 * tprime, qi, q, types[0]);
        if (sign1 == 1)
        {
            if (types[0] == S_SEG)
            {
                prim_val[0] = 0;
            }
            else if (types[0] == L_SEG)
            {
                prim_val[0] = 1;
            }
            else
            {
                prim_val[0] = 2;
            }
        }
        else
        {
            if (types[0] == S_SEG)
            {
                prim_val[0] = 3;
            }
            else if (types[0] == L_SEG)
            {
                prim_val[0] = 4;
            }
            else
            {
                prim_val[0] = 5;
            }
        }
        //cout<<"RS start->Stage 1"<<endl;
    }
    else if (tprime < (L1 + L2))
    {
        reedsshepp_segment(sign2 * (tprime - L1), q1, q, types[1]);
        if (sign2 == 1)
        {
            if (types[0] == S_SEG)
            {
                prim_val[0] = 0;
            }
            else if (types[0] == L_SEG)
            {
                prim_val[0] = 1;
            }
            else
            {
                prim_val[0] = 2;
            }
        }
        else
        {
            if (types[0] == S_SEG)
            {
                prim_val[0] = 3;
            }
            else if (types[0] == L_SEG)
            {
                prim_val[0] = 4;
            }
            else
            {
                prim_val[0] = 5;
            }
        }
        //cout<<"RS start->Stage 2"<<endl;
    }
    else if (tprime < (L1 + L2 + L3))
    {
        reedsshepp_segment(sign3 * (tprime - L1 - L2), q2, q, types[2]);
        if (sign3 == 1)
        {
            if (types[0] == S_SEG)
            {
                prim_val[0] = 0;
            }
            else if (types[0] == L_SEG)
            {
                prim_val[0] = 1;
            }
            else
            {
                prim_val[0] = 2;
            }
        }
        else
        {
            if (types[0] == S_SEG)
            {
                prim_val[0] = 3;
            }
            else if (types[0] == L_SEG)
            {
                prim_val[0] = 4;
            }
            else
            {
                prim_val[0] = 5;
            }
        }
        //cout<<"RS start->Stage 3"<<endl;
    }
    else if (tprime < (L1 + L2 + L3 + L4))
    {
        if (L4 > 1e-15)
        {
            reedsshepp_segment(sign4 * (tprime - L1 - L2 - L3), q3, q, types[3]);
            if (sign4 == 1)
            {
                if (types[0] == S_SEG)
                {
                    prim_val[0] = 0;
                }
                else if (types[0] == L_SEG)
                {
                    prim_val[0] = 1;
                }
                else
                {
                    prim_val[0] = 2;
                }
            }
            else
            {
                if (types[0] == S_SEG)
                {
                    prim_val[0] = 3;
                }
                else if (types[0] == L_SEG)
                {
                    prim_val[0] = 4;
                }
                else
                {
                    prim_val[0] = 5;
                }
            }
            //cout<<"RS start->Stage 4"<<endl;
        }
        else
        {

            //cout<<"RS stage 4 --- skip"<<endl;
        }
    }
    else
    {
        if (L5 > 1e-15)
        {
            reedsshepp_segment(sign5 * (tprime - L1 - L2 - L3 - L4), q4, q, types[4]);
            if (sign5 == 1)
            {
                if (types[0] == S_SEG)
                {
                    prim_val[0] = 0;
                }
                else if (types[0] == L_SEG)
                {
                    prim_val[0] = 1;
                }
                else
                {
                    prim_val[0] = 2;
                }
            }
            else
            {
                if (types[0] == S_SEG)
                {
                    prim_val[0] = 3;
                }
                else if (types[0] == L_SEG)
                {
                    prim_val[0] = 4;
                }
                else
                {
                    prim_val[0] = 5;
                }
            }
            //cout<<"RS start->Stage 5"<<endl;
            //cout<<tprime<<"     "<<(tprime-L1-L2-L3-L4)<<endl;
            //cout<<sign5 * (tprime-L1-L2-L3-L4)<<endl;
        }
        else
        {
            //cout<<"RS stage 5 --- skip"<<endl;
        }
    }

    // scale the target configuration, translate back to the original starting point
    q[0] = q[0] * path->rho + path->qi[0];
    q[1] = q[1] * path->rho + path->qi[1];
    q[2] = mod2pi(q[2]);

    return 0;
}

int reedsshepp_path_sample_many(ReedsSheppPath *path, ReedsSheppPathSamplingCallback cb, double stepSize, void *user_data)
{
    double x = 0.0;
    double length = reedsshepp_path_length(path);
    while (x < length)
    {
        double q[3];
        //reedsshepp_path_sample( path, x, q, prim_val );
        int retcode = cb(q, x, user_data);
        if (retcode != 0)
        {
            return retcode;
        }
        x += stepSize;
    }
    return 0;
}

int reedsshepp_path_endpoint(ReedsSheppPath *path, double q[3])
{
    // TODO - introduce a new constant rather than just using EPSILON
    //return reedsshepp_path_sample( path, reedsshepp_path_length(path) - EPSILON, q );
    return 0;
}

int reedsshepp_extract_subpath(ReedsSheppPath *path, double t, ReedsSheppPath *newpath)
{
    // calculate the true parameter
    double tprime = t / path->rho;

    // copy most of the data
    newpath->qi[0] = path->qi[0];
    newpath->qi[1] = path->qi[1];
    newpath->qi[2] = path->qi[2];
    newpath->rho = path->rho;
    newpath->type = path->type;

    // fix the parameters
    newpath->param[0] = fmin(path->param[0], tprime);
    newpath->param[1] = fmin(path->param[1], tprime - newpath->param[0]);
    newpath->param[2] = fmin(path->param[2], tprime - newpath->param[0] - newpath->param[1]);
    return 0;
}
} // namespace HybridAStar
#endif
