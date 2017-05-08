#ifndef KDT_WRAPPER
#define KDT_WRAPPER

#include "ANN/ANN.h"
#include "ANN/ANNx.h"
#include "ANN/ANNperf.h"

class KDTWrapper : public ANNkd_tree
{
public:
    KDTWrapper(ANNpointArray, int n, int dd, int bs=1, ANNsplitRule split=ANN_KD_SUGGEST);
    ~KDTWrapper();
    ANNpoint getRectLow();
    ANNpoint getRectHigh();
    bool isNeedFurtherSearch(ANNpoint query, ANNdist max_sq_dis);
};

#endif
