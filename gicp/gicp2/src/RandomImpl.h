#ifndef RANDOMIMPL_H
#define RANDOMIMPL_H

#include <boost/random.hpp>

class CRandomImpl
{
public:
    typedef boost::mt19937 RSeed;
    CRandomImpl(){}
    virtual ~CRandomImpl(){}
    double normal(double sigma)
    {
        return normal(0,sigma);
    }
    double normal(double mean, double sigma)
    {
        boost::normal_distribution<double> dist(mean, sigma);
        boost::variate_generator<RSeed&, boost::normal_distribution<double> > gen(rng, dist);
        return gen();
    }
    void seed(){ rng.seed(); }
private:
    RSeed rng;
};


#endif
