#include "PGicp.h"
#include <QMutexLocker>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <limits>
#include "globaldef.h"
#include <QThread>
#include <qtconcurrentrun.h>
#include <QtConcurrentMap> 
#include "boost/bind.hpp"

using namespace std;

CPGicp::CPGicp(int n_threads, double max_search_dis):
    m_num_threads(n_threads),
    m_max_search_sq(max_search_dis),
    m_kd_done(false)
{
    if(m_num_threads <= 0) m_num_threads = 2;
    if(m_max_search_sq <= 0) m_max_search_sq = 4;
}
CPGicp::~CPGicp(){}

void CPGicp::BuildKDTree()
{
    {
        QMutexLocker lock(&kdmutex);
        if(m_kd_done) return;
        m_kd_done = true;
    }
    
    int n = NumPoints();
    if(n == 0) return ;
    int range = n/m_num_threads;
    m_sub_range = range;
    m_pkdtree_array = new ANNpointArray[m_num_threads];
    
    int lstart = 0;
    int lstop = range;
    int subn;
    int index = 0;
    for(int i=0;i<m_num_threads;i++)
    {
        subn = lstop - lstart;
        ANNpointArray parray = annAllocPts(subn,3);
        for(int j=0; j<subn; j++)
        {
            parray[j][0] = point_[index].x;
            parray[j][1] = point_[index].y;
            parray[j][2] = point_[index].z;
            ++index;
        }
        m_pkdtree_array[i] = parray;
        KDTree* pKDtree = new KDTree(parray, subn, 3, 10);
        m_kdtreeList.push_back(new KD_Wrapper(pKDtree,i));
        lstart = index;
        if(index + range >= n) lstop = n;
        else lstop = index+range;
    }
}

void CPGicp::MergeTopK(ANNidx* nn_indecies, ANNdist* nn_dists, SearchResult* merged)
{
    int n = merged->n;
    int k = merged->k;
    // ANNdist * tmp = new ANNdist(k);
    static vector<ANNdist> tmp_dist(k);
    static vector<ANNidx> tmp_idx(k);
    if(tmp_idx.size()!=k)
    {
        tmp_idx.resize(k);
        tmp_dist.resize(k);
    }
    for(int item=0;item<n;item++)
    {
        int i = item*k;
        int j = i;
        int l=0;
        while(l<k)
        {
            if(nn_dists[i]<=merged->p_dist[j])
            {
                tmp_dist[l] = nn_dists[i];
                tmp_idx[l] = nn_indecies[i];
                i++;
            }else
            {
                tmp_dist[l] = merged->p_dist[j];
                tmp_idx[l] = BackIndex(merged->num_of_threads, merged->p_index[j]);
                j++;
            }
            l++;
        }
        if(j>i)
        {
            memcpy(&nn_indecies[item*k], &tmp_idx[0], k*sizeof(ANNidx));
            memcpy(&nn_dists[item*k], &tmp_dist[0], k*sizeof(ANNdist));
        }
    }
}

void CPGicp::SimpleMerge(ANNidx* nn_indecies, ANNdist* nn_dists, SearchResult* merged)
{
    int n = merged->n;
    for(int i=0;i<n;i++)
    {
        if(nn_dists[i] > merged->p_dist[i])
        {
            nn_dists[i] =  merged->p_dist[i];
            nn_indecies[i] = BackIndex(merged->num_of_threads, merged->p_index[i]);
        }
    }
}

void CPGicp::HandleResult(SearchResult* pRet, int num, ANNidx* nn_indecies, ANNdist* nn_dists )
{
    if(pRet == NULL)
    {
        cout<<"pRet is NULL!"<<endl;
        return ;
    }
    int n = pRet->n;
    int k = pRet->k;
    if(num==0)
    {
        memcpy(nn_dists, pRet->p_dist, n*k*sizeof(ANNdist));
        for(int j=0;j<n;j++)
        {
            for(int l=0;l<k;l++)
                nn_indecies[j*k+l] = BackIndex(pRet->num_of_threads, pRet->p_index[j*k+l]);
        }
    }else
    {
        if(k>1)
        {
            MergeTopK(nn_indecies, nn_dists, pRet);
        }else
        {
            SimpleMerge(nn_indecies, nn_dists, pRet);
        }
    }

}
CPGicp::SearchResult* CPGicp::knnPSearch(KD_Wrapper* KDT)
{
    if(KDT == 0)
    {
        cout<<"What's the matter!"<<endl;
        return 0;
    }
    cout<<"thread: "<<KDT->num_of_threads<<" is searching!"<<endl;
    if(KDT->bneedTrans)
    {
        return knnSearch(KDT, KDT->scan, KDT->trans, KDT->k, KDT->eps);
    }
    return knnSearch(KDT, KDT->scan, KDT->k, KDT->eps);
}


void CPGicp::knnPSearch(GpSet* scan, ANNidx* nn_indecies, ANNdist* nn_dists, dgc_transform_t& trans, int k, double eps )
{
    for(int i=0;i<m_kdtreeList.size();i++)
    {
        m_kdtreeList[i]->scan = scan;
        m_kdtreeList[i]->bneedTrans = true;
        dgc_transform_copy(m_kdtreeList[i]->trans, trans);
        m_kdtreeList[i]->k = k;
        m_kdtreeList[i]->eps = eps;
    }
    cout<<"before the search!"<<endl;
    QList<SearchResult*> results = QtConcurrent::blockingMapped(m_kdtreeList, boost::bind(&CPGicp::knnPSearch, this, _1));
    cout<<"after the search!"<<endl;
    for(int i=0;i<results.size();i++)
    {
        SearchResult* pRet = results[i];
        HandleResult(pRet, i, nn_indecies, nn_dists);
        delete pRet;
    }
    cout<<"finished search!"<<endl;
}

void CPGicp::knnPSearch(GpSet* scan, ANNidx* nn_indecies, ANNdist* nn_dists, int k, double eps)
{
    for(int i=0;i<m_kdtreeList.size();i++)
    {
        m_kdtreeList[i]->scan = scan;
        m_kdtreeList[i]->bneedTrans = false;
        m_kdtreeList[i]->k = k;
        m_kdtreeList[i]->eps = eps;
    }
    
    QList<SearchResult*> results = QtConcurrent::blockingMapped(m_kdtreeList, boost::bind(&CPGicp::knnPSearch, this, _1));
    for(int i=0;i<results.size();i++)
    {
        SearchResult* pRet = results[i];
        HandleResult(pRet, i, nn_indecies, nn_dists);
        delete pRet;
    }
}

void CPGicp::knnSearch(GpSet* scan, ANNidx* nn_indecies, ANNdist* nn_dists, dgc_transform_t& trans, int k, double eps)
{
   /* GpSet * tmp = new GpSet;
    double x,y,z;
    for(int i=0;i<scan->NumPoints();i++)
    {
        tmp->AppendPoint((*scan)[i]);
        dgc_transform_point(&((*tmp)[i].x),&((*tmp)[i].y),&((*tmp)[i].z),trans);
    }
    knnSearch(tmp, nn_indecies, nn_dists, k , eps);
    delete tmp;*/
    int n = scan->NumPoints();
    for(int i=0;i<m_num_threads;i++)
    {
        SearchResult* pRet = knnSearch(m_kdtreeList[i], scan, trans, k, eps);
        HandleResult(pRet, i, nn_indecies, nn_dists);
        delete pRet;
    }
    return ;
}

void CPGicp::knnSearch(dgc::gicp::GICPPointSet* scan, ANNidx* nn_indecies, ANNdist* nn_dists, int k, double eps)
{
    int n = scan->NumPoints();
    for(int i=0;i<m_num_threads;i++)
    {
        SearchResult* pRet = knnSearch(m_kdtreeList[i], scan, k, eps);
        HandleResult(pRet, i , nn_indecies, nn_dists);
        delete pRet;
    }
    return ;
}

CPGicp::SearchResult* CPGicp::knnSearch(KD_Wrapper* KDT, GpSet* scan, dgc_transform_t& trans, int k, double eps)
{
    if(k<1) return 0;
    SearchResult* ret = new SearchResult;
    int n = scan->NumPoints();
    ANNdist* nn_dist_sq = new ANNdist[n*k];
    ANNidx *nn_indecies = new ANNidx[n*k];
    ANNpoint query_point = annAllocPt(3);
    for(int i=0;i<n;i++)
    {
        query_point[0] = (*scan)[i].x;
        query_point[1] = (*scan)[i].y;
        query_point[2] = (*scan)[i].z;
        dgc_transform_point(&query_point[0], &query_point[1], &query_point[2], trans);
        /*if(!KDT->kdtree_->isNeedFurtherSearch(query_point, m_max_search_sq))
        {   
            // cout<<"does not need further search!"<<endl;
            for(int l=0; l<k; l++ )
            {
                nn_indecies[i*k+l] = 0;
                nn_dist_sq[i*k+l] = std::numeric_limits<double>::max(); // m_max_search_sq + 1;
            }
            continue;
        }*/
        KDT->kdtree_->annkSearch(query_point, k, &nn_indecies[i*k], &nn_dist_sq[i*k], eps);
    }

    ret->n = n;
    ret->k = k;
    ret->num_of_threads = KDT->num_of_threads;
    ret->p_dist = nn_dist_sq;
    ret->p_index = nn_indecies;
    return ret;
}

// to testify the correctness of this 
CPGicp::SearchResult* CPGicp::knnSearch(KD_Wrapper* KDT, dgc::gicp::GICPPointSet* scan, int k, double eps)
{
    if(k<1) return 0;
    SearchResult* ret = new SearchResult;
    int n = scan->NumPoints();
    ANNdist* nn_dist_sq = new ANNdist[n*k];
    ANNidx *nn_indecies = new ANNidx[n*k];
    ANNpoint query_point = annAllocPt(3);

    for(int i=0;i<n;i++)
    {
        query_point[0] = (*scan)[i].x;
        query_point[1] = (*scan)[i].y;
        query_point[2] = (*scan)[i].z;
        /*if(!KDT->kdtree_->isNeedFurtherSearch(query_point, m_max_search_sq))
        {
            for(int l=0; l<k; l++ )
            {
                nn_indecies[i*k+l] = -1;
                nn_dist_sq[i*k+l] = m_max_search_sq + 1;
            }
            continue;
        }*/
        KDT->kdtree_->annkSearch(query_point, k, &nn_indecies[i*k], &nn_dist_sq[i*k], eps);
    }

    ret->n = n;
    ret->k = k;
    ret->num_of_threads = KDT->num_of_threads;
    ret->p_dist = nn_dist_sq;
    ret->p_index = nn_indecies;
    return ret;
}

void TestPGicp(GpSet* p0, GpSet* p1)
{
    double max_d_sq = pow(2,2);
    cout<<"max_d_sq: "<<max_d_sq<<endl;

    dgc_transform_t trans;
    dgc_transform_identity(trans);
    dgc_transform_translate(trans, 0.3, -0.2, 1);
    // 1 GT corr
    cout<<"start GT corr!"<<endl;
    p0->BuildKDTree();
    int n = p1->NumPoints();
    int k = 1;
    ANNpoint query_point = annAllocPt(3);
    ANNidx *nn_indecies = new ANNidx[n*k];
    ANNdist* nn_dist = new ANNdist[k];
    ofstream gt_corr_out("gt_corr.log");
    ofstream my_corr_out("my_corr.log");
    StartTiming();
    for(int i=0;i<n;i++)
    {
        query_point[0] = (*p1)[i].x;
        query_point[1] = (*p1)[i].y;
        query_point[2] = (*p1)[i].z;
        dgc_transform_point(&query_point[0], &query_point[1], &query_point[2], trans);
        p0->kdtree_->annkSearch(query_point, k, &nn_indecies[i*k], nn_dist, 0.0);
        for(int j=0;j<k;j++)
        {
            if(nn_dist[j] < max_d_sq)
            {
                gt_corr_out<<i<<"\t"<<nn_indecies[i*k+j]<<endl;
            }
        }
    }
    double duration = StopTiming()*1000;
    cout<<"GT corr cost: "<<duration<<" ms!"<<endl;
    // 2 MY corr
    // get QT threads number
    QThreadPool* qtp = QThreadPool::globalInstance();
    if (qtp->maxThreadCount() - qtp->activeThreadCount() == 1) 
    {
        printf("Few Threads Remaining: Increasing maxThreadCount to %i", qtp->maxThreadCount()+1);
        qtp->setMaxThreadCount(qtp->maxThreadCount() + 1);
    }
    else{
        printf("current active threads: %i\n", qtp->activeThreadCount());
        printf("maxThreadCount threads: %i\n", qtp->maxThreadCount());
    }
    int n_threads = qtp->maxThreadCount() - qtp->activeThreadCount();
    cout<<"start My corr!"<<endl;
    CPGicp myicp(n_threads/2, max_d_sq);
    for(int i=0;i<p0->NumPoints();i++)
    {
        myicp.AppendPoint((*p0)[i]);
    }
    cout<<"start build my kdtree!"<<endl;
    myicp.BuildKDTree();
    cout<<"start to find all the corr!"<<endl;
    ANNidx* nn_idx2 = new ANNidx[n*k];
    ANNdist* nn_dist2 = new ANNdist[n*k];
    
    StartTiming();
    myicp.knnSearch(p1, nn_idx2, nn_dist2, trans, k);
    // myicp.knnPSearch(p1, nn_idx2, nn_dist2, trans, k);
    cout<<"start to write to log!"<<endl;
    for(int i=0;i<n;i++)
    {
        for(int j=0;j<k;j++)
        {
            if(nn_dist2[i*k+j] < max_d_sq)
            {
                my_corr_out<<i<<"\t"<<nn_idx2[i*k+j]<<endl;
            }
        }
    }
    duration = StopTiming()*1000;
    cout<<"my corr cost: "<<duration<<" ms!"<<endl;
    cout<<"FINISHED!"<<endl;
}



