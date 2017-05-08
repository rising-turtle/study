#include "globaldefinition.h"

int global_adjuster_max_keypoints = 1800;
int global_adjuster_min_keypoints = 800;//1000;
int global_fast_adjuster_max_iterations = 10;
int global_surf_adjuster_max_iterations = 5; //may slow down initially
///Ignorance w.r.t small motion (reduces redundancy)
float global_min_translation_meter = 0.1;//0.04;//0.1;
float global_min_rotation_degree = 5; 

///Maximally this many comparisons per node
///(lower=faster, higher=better loop closing)
unsigned int global_connectivity = 5;//10;
unsigned int global_potential_nodes = 20;

// spatial limitation for Nodes
double global_graph_radius = 2;
int global_graph_size = 500;
int global_bg_graph_threshold=40;
double global_max_translation_meter=8;
unsigned int global_min_inliers = 100;

// for log
int gl_pf_fid=0;
int gl_pf_gs=0;
double gl_pf_fe=0;
double gl_pf_fm=0;
double gl_pf_id=0;
double gl_pf_me=0;
double gl_pf_slam=0;
double gl_pf_start = 0;
double gl_pf_op=0;