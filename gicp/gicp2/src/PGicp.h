#ifndef PGICP_H
#define PGICP_H

#include "gicp.h"
#include "transform.h"
// #include "KDTWrapper.h"
#include <vector>
#include <Eigen/Core>
#include <QMutex>
#include <QList>

using namespace std;

typedef dgc::gicp::GICPPoint Gpt;
typedef dgc::gicp::GICPPointSet GpSet;
typedef ANNkd_tree KDTree;
// typedef KDTWrapper KDTree;

class CPGicp
{
    struct SearchResult
    {
        SearchResult():p_dist(0),p_index(0),n(-1){}
        ~SearchResult()
        {
            if(!p_dist) delete p_dist; 
            if(!p_index) delete p_index;
        }
        ANNdist* p_dist;
        ANNidx* p_index;
        int n;
        int k;
        int num_of_threads;
    };
    struct KD_Wrapper
    {
        KD_Wrapper(KDTree* kdtree, int num):kdtree_(kdtree), num_of_threads(num), bneedTrans(false){}
        ~KD_Wrapper(){if(!kdtree_) delete kdtree_; kdtree_ = NULL; }
        KDTree* kdtree_;
        int num_of_threads;
        // below : a patch to be used in qtconcurrent
        GpSet* scan;
        dgc_transform_t trans;
        bool bneedTrans;
        int k; 
        double eps;
    };
public:
    CPGicp(int n_threads = 2, double max_search_dis=4);
    virtual ~CPGicp();
    virtual SearchResult* knnSearch(KD_Wrapper*, dgc::gicp::GICPPointSet* , int k, double eps);
    virtual SearchResult* knnSearch(KD_Wrapper*, GpSet*, dgc_transform_t&, int k, double eps);

    virtual void knnSearch(dgc::gicp::GICPPointSet*, ANNidx*, ANNdist*, int k, double eps=0.);
    virtual void knnSearch(GpSet*, ANNidx*, ANNdist*, dgc_transform_t& , int k, double eps=0.);

    virtual void knnPSearch(GpSet*, ANNidx*, ANNdist*, int k, double eps=0.);
    virtual void knnPSearch(GpSet*, ANNidx*, ANNdist*, dgc_transform_t&, int k, double eps=0.);
    virtual SearchResult* knnPSearch(KD_Wrapper*);

    void MergeTopK(ANNidx*, ANNdist*, SearchResult*);
    void SimpleMerge(ANNidx*, ANNdist*, SearchResult*);
    virtual void BuildKDTree();
    inline size_t NumPoints(){return point_.size();}
    inline void AppendPoint(Gpt const & pt) { point_.push_back(pt); }
    inline int BackIndex(int num_threads, int num){return (num_threads*m_sub_range+num);}
    // virtual void ComputeMatrices();
public:
    QMutex kdmutex;
    int m_num_threads;
    int m_sub_range; 
    double m_max_search_sq;
    vector<Gpt> point_;
    bool m_kd_done;
    ANNpointArray * m_pkdtree_array;
    QList<KD_Wrapper*> m_kdtreeList;

protected:
    void HandleResult(SearchResult*, int, ANNidx* , ANNdist*);

private:
    CPGicp(const CPGicp&);
    CPGicp& operator=(const CPGicp&);
};

extern void TestPGicp(GpSet*, GpSet*);

#endif
