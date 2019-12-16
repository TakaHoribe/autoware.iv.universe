#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

class Polygon2d {
private:
  std::vector<Eigen::Vector2d> polygon_;

public:
  Polygon2d();
  Polygon2d(const std::vector<Eigen::Vector2d> &polygon);
  virtual ~Polygon2d(){};
  static bool isInPolygon(const std::vector<Eigen::Vector2d> &polygon, const Eigen::Vector2d &point);
  void setPolygon(const std::vector<Eigen::Vector2d> &polygon);
  bool isInPolygon(const Eigen::Vector2d &point);
  static bool isCollision(const std::vector<Eigen::Vector2d> &polygon, const std::vector<Eigen::Vector2d> &lines);
  static bool getCollisionPoints(const std::vector<Eigen::Vector2d> &_polygon,
                                  const std::vector<Eigen::Vector2d> &lines,
                                  std::vector<Eigen::Vector2d> &points);
  bool isCollision(const std::vector<Eigen::Vector2d> &lines);
  bool getCollisionPoints(const std::vector<Eigen::Vector2d> &lines,
                         std::vector<Eigen::Vector2d> &points);
};