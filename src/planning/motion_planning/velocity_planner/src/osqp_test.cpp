

void osqpTest()
{
// Solves convex quadratic programs (QPs) using the OSQP solver.
//
// The function returns a tuple containing the solution as two float vectors.
// The first element of the tuple contains the 'primal' solution. The second element contains the 'lagrange multiplier'
// solution.
//
// About OSQP  https://osqp.org/docs/
//
// Problem definition:
//  minimize    1/2 * xt * A * x + qt * x
//  subject to  l <= A * x <= u
//
// How to use:
//   1. Generate the Eigen matrices P, A and vectors q, l, u according to the problem.
//   2. Call the optimization function
//        Ex: std::tuple<std::vector<float>, std::vector<float>> result;
//            result = osqp::optimize(P, A, q, l, u);
//   3. Access the optimal parameters
//        Ex: std::vector<float> param = std::get<0>(result);




Eigen::MatrixXf A(3, 2);
Eigen::MatrixXf P(2, 2);
P << 1.0, -1.0, -1.0, 2.0;
A << 1,1,-1,2,2,1;


std::vector<float> q = {-2., -6.};
std::vector<float> l = {-10000.0, -10000.0};
std::vector<float> u = {2, 2, 3};

std::tuple<std::vector<float>, std::vector<float>> result;


auto t_start = std::chrono::system_clock::now();

result = osqp::optimize(P, A, q, l, u);

auto t_end = std::chrono::system_clock::now();
double elapsed_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_start).count() * 1.0e-6;
ROS_INFO("timer callback: calculation time = %f [ms]", elapsed_ms);

std::vector<float> param = std::get<0>(result);
printf("result = %f, %f\n", param[0], param[1]);





};
