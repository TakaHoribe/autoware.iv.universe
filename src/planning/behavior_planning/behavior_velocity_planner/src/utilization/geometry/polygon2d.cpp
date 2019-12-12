#include <utilization/geometry/polygon2d.hpp>
#include <iostream>

Polygon2d::Polygon2d() {}
Polygon2d::Polygon2d(const std::vector<Eigen::Vector2d> &polygon) : polygon_(polygon)
{
}

void Polygon2d::setPolygon(const std::vector<Eigen::Vector2d> &polygon)
{
    polygon_ = polygon;
    if (polygon_.front() != polygon_.back())
        polygon_.push_back(polygon_.front());
}

bool Polygon2d::isInPolygon(const std::vector<Eigen::Vector2d> &_polygon, const Eigen::Vector2d &point)
{
    if (_polygon.size() < 3)
    {
        std::cerr << __FILE__ << "(" << __LINE__ << ")"
                  << ":"
                  << "Not enough polygon points." << std::endl;
        return false;
    }
    std::vector<Eigen::Vector2d> polygon = _polygon;
    if (polygon.front() != polygon.back())
        polygon.push_back(polygon.front());

    // http://sampleyy.hatenablog.com/entry/2015/03/28/110152
    double signed_angle_sum = 0;
    for (int i = 0; i < (int)polygon.size() - 1; ++i)
    {
        Eigen::Vector2d vec1(polygon.at(i).x() - point.x(), polygon.at(i).y() - point.y());
        Eigen::Vector2d vec2(polygon.at(i + 1).x() - point.x(), polygon.at(i + 1).y() - point.y());
        const double dot = vec1.dot(vec2);
        const double cross = vec1.x() * vec2.y() - vec1.y() * vec2.x();
        double signed_angle = std::atan2(cross, dot);

        signed_angle_sum += signed_angle;
    }

    const double epsilon = 0.001;
    if ((2 * M_PI - std::fabs(signed_angle_sum)) < epsilon)
        return true;
    else
        return false;
}

bool Polygon2d::isInPolygon(const Eigen::Vector2d &point)
{
    return isInPolygon(polygon_, point);
}

bool Polygon2d::isCollision(const std::vector<Eigen::Vector2d> &_polygon, const std::vector<Eigen::Vector2d> &lines)
{
    if (_polygon.size() < 3)
    {
        std::cerr << __FILE__ << "(" << __LINE__ << ")"
                  << ":"
                  << "Not enough polygon points." << std::endl;
        return false;
    }
    std::vector<Eigen::Vector2d> polygon = _polygon;
    if (polygon.front() != polygon.back())
        polygon.push_back(polygon.front());

    const double epsilon = 0.001;
    double t1, t2, t3, t4;
    for (size_t i = 0; i < polygon.size() - 1; ++i)
    {
        for (size_t j = 0; j < lines.size() - 1; ++j)
        {
            t3 = (lines.at(j).x() - lines.at(j + 1).x()) *
                     (polygon.at(i).y() - lines.at(j).y()) +
                 (lines.at(j).y() - lines.at(j + 1).y()) *
                     (lines.at(j).x() - polygon.at(i).x());
            t4 = (lines.at(j).x() - lines.at(j + 1).x()) *
                     (polygon.at(i + 1).y() - lines.at(j).y()) +
                 (lines.at(j).y() - lines.at(j + 1).y()) *
                     (lines.at(j).x() - polygon.at(i + 1).x());
            if (t1 * t2 < 0 || std::fabs(t1) < epsilon || std::fabs(t2) < epsilon)
            {
                if (t3 * t4 < 0 || std::fabs(t3) < epsilon ||std::fabs(t4) < epsilon)
                {
                    return true;
                }
            }
        }
    }

    return false;
}
bool Polygon2d::getCollisionPoints(const std::vector<Eigen::Vector2d> &_polygon,
                                   const std::vector<Eigen::Vector2d> &lines,
                                   std::vector<Eigen::Vector2d> &_points)
{
    if (_polygon.size() < 3)
    {
        std::cerr << __FILE__ << "(" << __LINE__ << ")"
                  << ":"
                  << "Not enough polygon points." << std::endl;
        return false;
    }
    std::vector<Eigen::Vector2d> polygon = _polygon;
    if (polygon.front() != polygon.back())
        polygon.push_back(polygon.front());

    const double epsilon = 0.001;
    double t1, t2, t3, t4;
    std::vector<Eigen::Vector2d> points;
    for (size_t i = 0; i < polygon.size() - 1; ++i)
    {
        for (size_t j = 0; j < lines.size() - 1; ++j)
        {
            t1 = (polygon.at(i).x() - polygon.at(i + 1).x()) *
                     (lines.at(j).y() - polygon.at(i).y()) +
                 (polygon.at(i).y() - polygon.at(i + 1).y()) *
                     (polygon.at(i).x() - lines.at(j).x());
            t2 = (polygon.at(i).x() - polygon.at(i + 1).x()) *
                     (lines.at(j + 1).y() - polygon.at(i).y()) +
                 (polygon.at(i).y() - polygon.at(i + 1).y()) *
                     (polygon.at(i).x() - lines.at(j + 1).x());
            t3 = (lines.at(j).x() - lines.at(j + 1).x()) *
                     (polygon.at(i).y() - lines.at(j).y()) +
                 (lines.at(j).y() - lines.at(j + 1).y()) *
                     (lines.at(j).x() - polygon.at(i).x());
            t4 = (lines.at(j).x() - lines.at(j + 1).x()) *
                     (polygon.at(i + 1).y() - lines.at(j).y()) +
                 (lines.at(j).y() - lines.at(j + 1).y()) *
                     (lines.at(j).x() - polygon.at(i + 1).x());
            if (t1 * t2 < 0 || std::fabs(t1) < epsilon || std::fabs(t2) < epsilon)
            {
                if (t3 * t4 < 0 || std::fabs(t3) < epsilon ||std::fabs(t4) < epsilon)
                {
                    // std::cout << polygon.at(i).x() << ", " << polygon.at(i).y() << std::endl;
                    // std::cout << polygon.at(i + 1).x() << ", " << polygon.at(i + 1).y() << std::endl;
                    // std::cout << lines.at(j).x() << ", " << lines.at(j).y() << std::endl;
                    // std::cout << lines.at(j + 1).x() << ", " << lines.at(j + 1).y() << std::endl;
                    // std::cout << t1 << ", " << t2 << std::endl;
                    // std::cout << "---------------" << std::endl;
                    const double a1 = (polygon.at(i + 1).y() - polygon.at(i).y()) / (polygon.at(i + 1).x() - polygon.at(i).x());
                    const double b1 = polygon.at(i).y() - a1 * polygon.at(i).x();
                    const double a2 = (lines.at(j + 1).y() - lines.at(j).y()) / (lines.at(j + 1).x() - lines.at(j).x());
                    const double b2 = lines.at(j + 1).y() - a2 * lines.at(j + 1).x();
                    Eigen::Vector2d point;
                    point << (b2 - b1) / (a1 - a2), (a1 * b2 - a2 * b1) / (a1 - a2);
                    points.push_back(point);
                }
            }
        }
    }
    if (points.empty())
        return false;
    else
    {
        _points = points;
        return true;
    }
}

bool Polygon2d::isCollision(const std::vector<Eigen::Vector2d> &lines)
{
    return isCollision(polygon_, lines);
}
bool Polygon2d::getCollisionPoints(const std::vector<Eigen::Vector2d> &lines,
                                  std::vector<Eigen::Vector2d> &points)
{
    return getCollisionPoints(polygon_, lines, points);
}
