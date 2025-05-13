#ifndef SLAM_H
#define SLAM_H

#include <Eigen/Dense>
#include <vector>

// ----- Utility functions -----

// Build 2D rotation matrix
Eigen::Matrix2d rotate(double radians);

// Apply rotation + translation to a 2D point
Eigen::Vector2d transformation_matrix_point(const Eigen::Matrix2d &r,
                                            const Eigen::Vector2d &t,
                                            const Eigen::Vector2d &p);

// ----- Submap class -----

class Submap {
public:
    Submap(double resolution, int width, int height, double p_min, double p_max);

    void insertHit(const Eigen::Vector2d& world_point);
    void insertMiss(const Eigen::Vector2d& world_point);
    double getProbability(int i, int j) const;

private:
    double resolution_;
    int width_, height_;
    double p_min_, p_max_;
    Eigen::MatrixXd grid_;

    Eigen::Vector2i worldToGrid(const Eigen::Vector2d& world_point) const;
    double odds(double p) const;
    double oddsInv(double o) const;
};

#endif // SLAM_H
