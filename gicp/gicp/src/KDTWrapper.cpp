#include "KDTWrapper.h"

KDTWrapper::KDTWrapper(ANNpointArray pa, int n, int dd, int bs, ANNsplitRule split):
    ANNkd_tree(pa, n, dd, bs, split)
{}
KDTWrapper::~KDTWrapper(){}

ANNpoint KDTWrapper::getRectLow(){ return bnd_box_lo;}
ANNpoint KDTWrapper::getRectHigh(){return bnd_box_hi;}

bool KDTWrapper::isNeedFurtherSearch(ANNpoint query, ANNdist max_sq_dis)
{
    ANNpoint lo = getRectLow();
    ANNpoint hi = getRectHigh();

    register ANNdist dist;              // distance to data point
    register ANNcoord t;
 
    dist = 0;
    for(int i=0;i<dim;i++)
    {
        if(query[i] < lo[i]) 
        {
            t = lo[i] - query[i];
        }else if(query[i] > hi[i] )
        {
            t = query[i] - hi[i];
        }else
        {
            continue;
        }
        if( (dist = ANN_SUM(dist, ANN_POW(t))) >= max_sq_dis) 
        {
            return false;
        }
    }
    return true;
}
