#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

class Line2d {
private:
  Eigen::Vector2d vec1_;
  Eigen::Vector2d vec2_;
public:
  Line2d();
  Line2d(const Eigen::Vector2d &point1, const Eigen::Vector2d &point2);
  virtual ~Line2d(){};
  static bool getBackwordPointFromBasePoint(const Eigen::Vector2d &line_point1,
                                            const Eigen::Vector2d &line_point2,
                                            const Eigen::Vector2d &base_point,
                                            const double backward_length,
                                            Eigen::Vector2d &output_point);
};