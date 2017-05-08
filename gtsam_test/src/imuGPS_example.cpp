
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <fstream>
#include <iostream>


using namespace gtsam; 
using namespace std; 

using symbol_shorthand::X; // Pose3 (x, y, z, r, p, y)
using symbol_shorthand::V; // Vel (xdot, ydot, zdot)
using symbol_shorthand::B; // Bias (ax, ay, az, gx, gy, gz) 

// typedef Eigen::Matrix<double, 3, 3> Matrix33;

int main(int argc, char* argv[])
{
  string data_file = ""; 
  
  // parse data file
  ifstream inf(data_file.c_str()); 
  string value; 

  // Format is (N, E, D, qX, qY, qZ, qW, velN, velE, velD)
  Eigen::Matrix<double, 10, 1> initial_state = Eigen::Matrix<double, 10, 1>::Zero(); 
  getline(inf, value, ','); // get rid of i
  for(int i=0; i<9; i++)
  {
    getline(inf, value, ',');
    initial_state(i) = atof(value.c_str()); 
  }
  getline(inf, value, '\n');
  initial_state(9) = atof(value.c_str()); 
  cout<<"initial state:\n "<< initial_state.transpose()<<"\n\n"; 
  
  // Assemble initial quaternion through gtsam constructor ::quaternion(qw, qx, qy, qz) 
  Rot3 prior_rotation = Quaternion(initial_state(6), initial_state(3), initial_state(4), initial_state(5)); 
  Point3 prior_point(initial_state.head<3>());
  Pose3 prior_pose(prior_rotation, prior_point); 
  Vector3 prior_velocity(initial_state.tail<3>()); 
  
  imuBias::ConstantBias prior_imu_bias; // assume zero initial bias 
  
  Values initial_values; 
  int correction_cnt = 0; 
  initial_values.insert(X(correction_cnt), prior_pose); 
  initial_values.insert(V(correction_cnt), prior_velocity); 
  initial_values.insert(B(correction_cnt), prior_imu_bias); 

  // Assemble prior noise model and add it in the graph 
  noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6)<< 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad, rad, rad, m, m, m
  noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.1); // m/s 
  noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);  

  // Add all prior factors (pose velocity bias) to the graph 
  NonlinearFactorGraph * graph = new NonlinearFactorGraph(); 
  graph->add(PriorFactor<Pose3>(X(correction_cnt), prior_pose, pose_noise_model)); 
  graph->add(PriorFactor<Vector3>(V(correction_cnt), prior_velocity, velocity_noise_model)); 
  graph->add(PriorFactor<imuBias::ConstantBias>(B(correction_cnt), prior_imu_bias, bias_noise_model));

  // use the sensor specs to build the noise model for the IMU factor 
  double accel_noise_sigma = 0.0003924; 
  double gyro_noise_sigma = 0.000205689024915; 
  double accel_bias_rw_sigma = 0.004905; 
  double gyro_bias_rw_sigma = 0.000001454441043; 
  Matrix33 measured_acc_cov = Matrix33::Identity(3,3)*pow(accel_noise_sigma, 2); 
  Matrix33 measured_omega_cov = Matrix33::Identity(3,3)*pow(gyro_noise_sigma, 2); 
  Matrix33 integration_error_cov = Matrix33::Identity(3,3)*1e-8; // error committed in integration position from veloctiy
  Matrix33 bias_acc_cov = Matrix33::Identity(3,3) * pow(accel_bias_rw_sigma, 2);
  Matrix33 bias_omega_cov = Matrix33::Identity(3,3) * pow(gyro_bias_rw_sigma, 2);
  Matrix66 bias_acc_omega_int = Matrix::Identity(6,6) * 1e-5; // error in the bias used for preintegration 

  boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0); 
  p->accelerometerCovariance = measured_acc_cov; 
  p->integrationCovariance = integration_error_cov; 
  p->gyroscopeCovariance = measured_omega_cov; 
  p->biasAccCovariance = bias_acc_cov; 
  p->biasOmegaCovariance = bias_omega_cov; 
  p->biasAccOmegaInt = bias_acc_omega_int; 
  
  PreintegratedCombinedMeasurements * p_combined_pre_imu = new PreintegratedCombinedMeasurements(p, prior_imu_bias);
  
  // CombinedImuFactor::CombinedPreintegratedMeasurements* p_combined_pre_imu = new CombinedImuFactor::CombinedPreintegratedMeasurements(prior_imu_bias, measured_acc_cov, measured_omega_cov, integration_error_cov, bias_acc_cov, bias_omega_cov, bias_acc_omega_int); 
  
  // Store previous state for the imu preintegration and the latest predicted outcome 
  NavState prev_state(prior_pose, prior_velocity); 
  NavState prop_state = prev_state; 
  imuBias::ConstantBias prev_bias = prior_imu_bias; 

  // Keep track of the total error over the entire run 
  double current_position_error = 0; double current_rotation_error = 0; 
  double dt = 0.005; 
  
  // let's do it 
  while(inf.good())
  {
    // parse out first value 
    getline(inf, value, ','); 
    int type = atoi(value.c_str()); 
    if(type == 0) // IMU data 
    { 
      Eigen::Matrix<double, 6, 1> imu = Eigen::Matrix<double, 6, 1>::Zero(); 
      for(int i=0; i<5; i++)
      {
        getline(inf, value, ','); 
        imu(i) = atof(value.c_str()); 
      }
      getline(inf, value,'\n'); 
      imu(5) = atof(value.c_str()); 

      // Adding the IMU preintegration 
      p_combined_pre_imu->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt); 

    }else if(type == 1) // GPS data 
    { 
      Eigen::Matrix<double, 7,1> gps = Eigen::Matrix<double, 7,1>::Zero(); 
      for(int i=0; i<6; i++)
      {
        getline(inf, value, ',');
        gps(i) = atof(value.c_str()); 
      }
      getline(inf, value, '\n'); 
      gps(6) = atof(value.c_str()); 
      
      correction_cnt ++ ; // this step requires optimization 
      
      CombinedImuFactor imu_factor(X(correction_cnt-1), V(correction_cnt-1), 
                                    X(correction_cnt), V(correction_cnt), 
                                    B(correction_cnt-1), B(correction_cnt), * p_combined_pre_imu);
      graph->add(imu_factor); 

      noiseModel::Diagonal::shared_ptr correction_noise = noiseModel::Isotropic::Sigma(3, 1.); 
      GPSFactor gps_factor(X(correction_cnt), Point3(gps(0), gps(1), gps(2)), correction_noise); 
      graph->add(gps_factor); 
      
      // Now optimize and compare results 
      prop_state = p_combined_pre_imu->predict(prev_state, prev_bias); 
      initial_values.insert(X(correction_cnt), prop_state.pose()); 
      initial_values.insert(V(correction_cnt), prop_state.v()); 
      initial_values.insert(B(correction_cnt), prev_bias); 
      
      LevenbergMarquardtOptimizer optimizer(*graph, initial_values); 
      Values result = optimizer.optimize(); 

      // Overwrite the beginning of the preintegration for the next step       
      prev_state = NavState(result.at<Pose3>(X(correction_cnt)), result.at<Vector3>(V(correction_cnt))); 
      prev_bias =   result.at<imuBias::ConstantBias>(B(correction_cnt));
      
      // Reset the preintegration object 
      p_combined_pre_imu->resetIntegrationAndSetBias(prev_bias); 
      
      // Print out the position and orientation error for comparison 
      Vector3 gtsam_position = prev_state.pose().translation(); 
      Vector3 position_error = gtsam_position - gps.head<3>(); 
      current_position_error = position_error.norm(); 
  
      Quaternion gtsam_quat = prev_state.pose().rotation().toQuaternion(); 
      Quaternion gps_quat(gps(6), gps(3), gps(4), gps(5)); 
      Quaternion quat_error = gtsam_quat * gps_quat.inverse(); 
      quat_error.normalize(); 
      Vector3 euler_angle_error(quat_error.x()*2, quat_error.y()*2, quat_error.z()*2); 
      current_rotation_error = euler_angle_error.norm();
      
      cout<<"Position error: "<<current_position_error <<" \t "<< " Angular error: "<<current_rotation_error<<"\n"; 


    }else{
    
      cout<<"Parsing file error!\n"; 
      return 1;
    }
  }
  
  inf.close();
  cout<<"Complete !\n";
  return 0; 

}
