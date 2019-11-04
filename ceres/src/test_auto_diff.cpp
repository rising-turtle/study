

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ceres/ceres.h>

using namespace ceres;

struct CostFunctor {
    template <typename T>
	bool operator()(const T* const x, T* residual) const {
	    // residual[0] = 10.0 - x[0];
	    typename Eigen::Map<Eigen::Matrix<T, 2, 1>> res(residual); 
	    typename Eigen::Matrix<T, 2, 1> target(T(10.), T(20.)); 
	    // Eigen::Matrix<double, 2, 1> target(10., 20.); 
	    typename Eigen::Matrix<T, 2, 1> vx(x[0], x[1]); 
	    res = target - vx; 
	    // residual[0] = 10.0 - x[0]; 
	    // residual[1] = 3.0 - x[1];
	    return true;
	}
};



int main(int argc, char** argv) {
    // google::InitGoogleLogging(argv[0]);

    // The variable to solve for with its initial value.
    Eigen::Matrix<double, 2, 1> initial_x(5.0, 3.0); 
    // double initial_x = 5.0;
    // double x = initial_x;
    double x[2] = {5.0, 3.0}; 

    // Build the problem.
    Problem problem;

    // Set up the only cost function (also known as residual). This uses
    // auto-differentiation to obtain the derivative (jacobian).
    CostFunction* cost_function =
	new AutoDiffCostFunction<CostFunctor, 2, 2>(new CostFunctor);
    problem.AddResidualBlock(cost_function, NULL, x);

    // Run the solver!
    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x
	<< " -> " << x[0]<<" "<<x[1] << "\n";
    return 0;
}
