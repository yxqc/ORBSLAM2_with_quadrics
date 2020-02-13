/**
* This file is part of CubeSLAM
*
* Copyright (C) 2018  Shichao Yang (Carnegie Mellon Univ)
*/

#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "quadric_slam/include/matrix_utils.h"

#include "quadric_slam/include/g2o_Object.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <math.h>
#include <algorithm> // std::swap

namespace g2o
{

using namespace Eigen;
using namespace std;

SE3Quat exptwist_norollpitch(const Vector6d &update)
{
    Vector3d omega;
    for (int i = 0; i < 3; i++)
        omega[i] = update[i];
    Vector3d upsilon;
    for (int i = 0; i < 3; i++)
        upsilon[i] = update[i + 3];

    double theta = omega.norm();
    Matrix3d Omega = skew(omega);

    Matrix3d R;
    R << cos(omega(2)), -sin(omega(2)), 0,
        sin(omega(2)), cos(omega(2)), 0,
        0, 0, 1;

    Matrix3d V;
    if (theta < 0.00001)
    {
        V = R;
    }
    else
    {
        Matrix3d Omega2 = Omega * Omega;

        V = (Matrix3d::Identity() + (1 - cos(theta)) / (theta * theta) * Omega + (theta - sin(theta)) / (pow(theta, 3)) * Omega2);
    }

    return SE3Quat(Quaterniond(R), V * upsilon);
}

void VertexQuadric::oplusImpl(const double *update_)
{
    Eigen::Map<const Vector9d> update(update_);

    g2o::Quadric newQuadric;
    if (whether_fixrotation)
    {
        newQuadric.pose.setTranslation(_estimate.pose.translation() + update.segment<3>(3));
    }
    else if (whether_fixrollpitch) //NOTE this only works for Quadric already has parallel to ground. otherwise update_z will also change final RPY
    {
        Vector9d update2 = update;
        update2(0) = 0;
        update2(1) = 0;
        newQuadric.pose = _estimate.pose * exptwist_norollpitch(update2.head<6>()); //NOTE object pose is from object to world!!!!
    }
    else
        newQuadric.pose = _estimate.pose * SE3Quat::exp(update.head<6>());

    if (whether_fixheight) // use previous height
        newQuadric.setTranslation(Vector3d(newQuadric.translation()(0), _estimate.translation()(1), newQuadric.translation()(2)));

    if (fixedscale(0) > 0) // if fixed scale is set, use it.
        newQuadric.scale = fixedscale;
    else
        newQuadric.scale = _estimate.scale + update.tail<3>();

    setEstimate(newQuadric);
}

// similar as above
void VertexQuadricFixScale::oplusImpl(const double *update_)
{
    Eigen::Map<const Vector6d> update(update_);

    g2o::Quadric newQuadric;
    if (whether_fixrotation)
    {
        newQuadric.pose.setRotation(_estimate.pose.rotation());
        newQuadric.pose.setTranslation(_estimate.pose.translation() + update.tail<3>());
    }
    else if (whether_fixrollpitch)
    {
        Vector6d update2 = update;
        update2(0) = 0;
        update2(1) = 0;
        newQuadric.pose = _estimate.pose * exptwist_norollpitch(update2);
    }
    else
        newQuadric.pose = _estimate.pose * SE3Quat::exp(update);

    if (whether_fixheight)
        newQuadric.setTranslation(Vector3d(newQuadric.translation()(0), _estimate.translation()(1), newQuadric.translation()(2)));

    if (fixedscale(0) > 0)
        newQuadric.scale = fixedscale;
    else
        newQuadric.scale = _estimate.scale;

    setEstimate(newQuadric);
}

void EdgeSE3QuadricFixScaleProj::computeError()
{
    const VertexSE3Expmap *SE3Vertex = dynamic_cast<const VertexSE3Expmap *>(_vertices[0]);                 //  world to camera pose
    const VertexQuadricFixScale *QuadricVertex = dynamic_cast<const VertexQuadricFixScale *>(_vertices[1]); //  object pose to world

    SE3Quat cam_pose_Tcw = SE3Vertex->estimate();
    Quadric global_Quadric = QuadricVertex->estimate();

    Vector4d rect_project = global_Quadric.projectOntoImageBbox(cam_pose_Tcw, Kalib); // center, width, height
    _error = rect_project - _measurement;
}

double EdgeSE3QuadricFixScaleProj::get_error_norm()
{
    computeError();
    return _error.norm();
}

void EdgeSE3QuadricProj::computeError()
{

    //    std::cout << "EdgeSE3QuadricProj computeError" << std::endl;
    const VertexSE3Expmap *SE3Vertex = static_cast<const VertexSE3Expmap *>(
        _vertices[0]); //  world to camera pose
    const VertexQuadric *quadricVertex = static_cast<const VertexQuadric *>(
        _vertices[1]); //  object pose to world

    SE3Quat cam_pose_Tcw = SE3Vertex->estimate();
    Quadric global_quadric = quadricVertex->estimate();

    Vector4d rect_project = global_quadric.projectOntoImageBbox(
        cam_pose_Tcw, calib); // Attention：center, width, height

    _error = (rect_project - _measurement).array().pow(2);
}

double EdgeSE3QuadricProj::get_error_norm()
{
    computeError();
    return _error.norm();
}

void EdgeDynamicPointQuadricCamera::computeError()
{
    const VertexSE3Expmap *SE3Vertex = dynamic_cast<const VertexSE3Expmap *>(_vertices[0]);                 // world to camera pose
    const VertexQuadricFixScale *QuadricVertex = dynamic_cast<const VertexQuadricFixScale *>(_vertices[1]); // object to world pose
    const VertexSBAPointXYZ *pointVertex = dynamic_cast<const VertexSBAPointXYZ *>(_vertices[2]);           // point to object pose

    Vector3d localpt = SE3Vertex->estimate() * (QuadricVertex->estimate().pose * pointVertex->estimate());

    Vector2d projected = Vector2d(Kalib(0, 2) + Kalib(0, 0) * localpt(0) / localpt(2), Kalib(1, 2) + Kalib(1, 1) * localpt(1) / localpt(2));
    _error = _measurement - projected;
}

void EdgeDynamicPointQuadricCamera::linearizeOplus()
{
    const VertexSE3Expmap *SE3Vertex = dynamic_cast<const VertexSE3Expmap *>(_vertices[0]);                 // world to camera pose
    const VertexQuadricFixScale *QuadricVertex = dynamic_cast<const VertexQuadricFixScale *>(_vertices[1]); // object to world pose
    const VertexSBAPointXYZ *pointVertex = dynamic_cast<const VertexSBAPointXYZ *>(_vertices[2]);           // point to object pose

    Vector3d objectpt = pointVertex->estimate();
    SE3Quat combinedT = SE3Vertex->estimate() * QuadricVertex->estimate().pose;
    Vector3d camerapt = combinedT * objectpt;

    double fx = Kalib(0, 0);
    double fy = Kalib(1, 1);

    double x = camerapt[0];
    double y = camerapt[1];
    double z = camerapt[2];
    double z_2 = z * z;

    Matrix<double, 2, 3> projptVscamerapt; //2d projected pixel / 3D local camera pt
    projptVscamerapt(0, 0) = fx / z;
    projptVscamerapt(0, 1) = 0;
    projptVscamerapt(0, 2) = -x * fx / z_2;

    projptVscamerapt(1, 0) = 0;
    projptVscamerapt(1, 1) = fy / z;
    projptVscamerapt(1, 2) = -y * fy / z_2;

    // jacobian of point
    _jacobianOplus[2] = -projptVscamerapt * combinedT.rotation().toRotationMatrix();

    // jacobian of camera
    _jacobianOplus[0](0, 0) = x * y / z_2 * fx;
    _jacobianOplus[0](0, 1) = -(1 + (x * x / z_2)) * fx;
    _jacobianOplus[0](0, 2) = y / z * fx;
    _jacobianOplus[0](0, 3) = -1. / z * fx;
    _jacobianOplus[0](0, 4) = 0;
    _jacobianOplus[0](0, 5) = x / z_2 * fx;

    _jacobianOplus[0](1, 0) = (1 + y * y / z_2) * fy;
    _jacobianOplus[0](1, 1) = -x * y / z_2 * fy;
    _jacobianOplus[0](1, 2) = -x / z * fy;
    _jacobianOplus[0](1, 3) = 0;
    _jacobianOplus[0](1, 4) = -1. / z * fy;
    _jacobianOplus[0](1, 5) = y / z_2 * fy;

    // jacobian of object pose.   obj twist  [angle position]
    Matrix<double, 3, 6> skewjaco;
    skewjaco.leftCols<3>() = -skew(objectpt);
    skewjaco.rightCols<3>() = Matrix3d::Identity();
    _jacobianOplus[1] = _jacobianOplus[2] * skewjaco; //2*6
    if (QuadricVertex->whether_fixrollpitch)          //zero gradient for roll/pitch
    {
        _jacobianOplus[1](0, 0) = 0;
        _jacobianOplus[1](0, 1) = 0;
        _jacobianOplus[1](1, 0) = 0;
        _jacobianOplus[1](1, 1) = 0;
    }
    if (QuadricVertex->whether_fixrotation)
    {
        _jacobianOplus[1](0, 0) = 0;
        _jacobianOplus[1](0, 1) = 0;
        _jacobianOplus[1](1, 0) = 0;
        _jacobianOplus[1](1, 1) = 0;
        _jacobianOplus[1](0, 2) = 0;
        _jacobianOplus[1](1, 2) = 0;
    }
}

Vector2d EdgeDynamicPointQuadricCamera::computeError_debug()
{
    computeError();
    return _error;
}

void EdgeObjectMotion::computeError()
{
    const VertexQuadricFixScale *QuadricVertexfrom = dynamic_cast<const VertexQuadricFixScale *>(_vertices[0]); // object to world pose
    const VertexQuadricFixScale *QuadricVertexto = dynamic_cast<const VertexQuadricFixScale *>(_vertices[1]);   // object to world pose
    const VelocityPlanarVelocity *velocityVertex = dynamic_cast<const VelocityPlanarVelocity *>(_vertices[2]);  // object to world pose

    if (QuadricVertexfrom == nullptr || QuadricVertexto == nullptr || velocityVertex == nullptr)
        cout << "Bad casting when compute Edge motion error!!!!!!!!!!!!!" << endl;

    // predict motion x y yaw and compute measurement.
    SE3Quat posefrom = QuadricVertexfrom->estimate().pose;
    double yaw_from = posefrom.toXYZPRYVector()(5);

    SE3Quat poseto = QuadricVertexto->estimate().pose;
    double yaw_to = poseto.toXYZPRYVector()(5);

    Vector2d velocity = velocityVertex->estimate(); //v w   linear velocity and steer angle

    const double vehicle_length = 2.71; // front and back wheels distance
    // vehicle motion model is applied to back wheel center
    Vector3d trans_back_pred = posefrom.translation() + (velocity(0) * delta_t - vehicle_length * 0.5) * Vector3d(cos(yaw_from), sin(yaw_from), 0);
    double yaw_pred = yaw_from + tan(velocity(1)) * delta_t / vehicle_length * velocity(0);

    // as mentioned in paper: my object frame is at the center. the motion model applies to back wheen center. have offset.
    Vector3d trans_pred = trans_back_pred + vehicle_length * 0.5 * Vector3d(cos(yaw_pred), sin(yaw_pred), 0);

    _error = Vector3d(poseto.translation()[0], poseto.translation()[1], yaw_to) - Vector3d(trans_pred(0), trans_pred(1), yaw_pred);
    if (_error[2] > 2.0 * M_PI)
        _error[2] -= 2.0 * M_PI;
    if (_error[2] < -2.0 * M_PI)
        _error[2] += 2.0 * M_PI;
}

Vector3d EdgeObjectMotion::computeError_debug()
{
    computeError();
    return _error;
}

Vector3d Quadric::point_boundary_error(const Vector3d &world_point, const double max_outside_margin_ratio, double point_scale) const
{
    // transform the point to local object frame  TODO actually can compute gradient analytically...
    Vector3d local_pt = point_scale * (this->pose.inverse() * world_point).cwiseAbs(); // change global point to local Quadric body frame.  make it positive.
    Vector3d error;

    // if point is within the Quadric, error=0, otherwise penalty how far it is outside Quadric
    for (int i = 0; i < 3; i++)
    {
        if (local_pt(i) < this->scale(i))
            error(i) = 0;
        else if (local_pt(i) < (max_outside_margin_ratio + 1) * this->scale(i))
            error(i) = local_pt(i) - this->scale(i);
        else
            error(i) = max_outside_margin_ratio * this->scale(i); // if points two far, give a constant error, don't optimize.
    }

    return error;
}

void EdgePointQuadricOnlyObject::computeError()
{
    const VertexQuadric *QuadricVertex = dynamic_cast<const VertexQuadric *>(_vertices[0]); // world to camera pose

    _error.setZero();

    const g2o::Quadric &estimate_Quadric = QuadricVertex->estimate();

    Vector3d point_edge_error;
    point_edge_error.setZero();
    for (size_t i = 0; i < object_points.size(); i++) // use abs  otherwise   pos neg will counteract by different pts.     maybe each edge one pt?
        point_edge_error += estimate_Quadric.point_boundary_error(object_points[i], max_outside_margin_ratio).cwiseAbs();
    if (object_points.size() > 0)
        point_edge_error = point_edge_error / object_points.size();

    point_edge_error = point_edge_error.array() / estimate_Quadric.scale.array(); //scale it

    // add prior shape dimension error?
    double prior_weight = 0.2;
    Vector3d prior_shape_error = estimate_Quadric.scale; // setZero?  or penalize large box! or set a range?
    if (prior_object_half_size(0) > 0)                   // if prior shape is being set, such as KITTI, then give large weight for shape error
    {
        prior_weight = 50.0;
        prior_shape_error = ((estimate_Quadric.scale - prior_object_half_size).array() / prior_object_half_size.array()).cwiseAbs();
    }

    _error = 1.0 * point_edge_error + prior_weight * prior_shape_error;
}

Vector3d EdgePointQuadricOnlyObject::computeError_debug()
{
    computeError();
    return _error;
}

// similar as above
void EdgePointQuadricOnlyObjectFixScale::computeError()
{
    const VertexQuadricFixScale *QuadricVertex = dynamic_cast<const VertexQuadricFixScale *>(_vertices[0]); // world to camera pose

    _error.setZero();

    const g2o::Quadric &estimate_Quadric = QuadricVertex->estimate();

    Vector3d point_edge_error;
    point_edge_error.setZero();
    for (size_t i = 0; i < object_points.size(); i++)
        point_edge_error += estimate_Quadric.point_boundary_error(object_points[i], max_outside_margin_ratio).cwiseAbs();
    if (object_points.size() > 0)
        point_edge_error = point_edge_error / object_points.size();

    point_edge_error = point_edge_error.array() / estimate_Quadric.scale.array();

    _error = 1.0 * point_edge_error;
}

void EdgePointQuadric::computeError()
{
    const VertexSBAPointXYZ *pointVertex = dynamic_cast<const VertexSBAPointXYZ *>(_vertices[0]); // point position
    const VertexQuadric *QuadricVertex = dynamic_cast<const VertexQuadric *>(_vertices[1]);       //  object pose to world
    const g2o::Quadric estimate_Quadric = QuadricVertex->estimate();

    Vector3d point_edge_error = estimate_Quadric.point_boundary_error(pointVertex->estimate(), max_outside_margin_ratio).cwiseAbs(); // abs to add shape error
    point_edge_error = point_edge_error.array() / estimate_Quadric.scale.array();

    // add prior shape dimension error?
    double prior_weight = 0.2;
    Vector3d prior_shape_error = estimate_Quadric.scale; // setZero?  or penalize large box! or set a range?
    _error = 1.0 * point_edge_error + prior_weight * prior_shape_error;
}

void EdgePointQuadricFixScale::computeError()
{
    const VertexSBAPointXYZ *pointVertex = dynamic_cast<const VertexSBAPointXYZ *>(_vertices[0]);           // point position
    const VertexQuadricFixScale *QuadricVertex = dynamic_cast<const VertexQuadricFixScale *>(_vertices[1]); //  object pose to world
    const g2o::Quadric estimate_Quadric = QuadricVertex->estimate();

    Vector3d point_edge_error = estimate_Quadric.point_boundary_error(pointVertex->estimate(), max_outside_margin_ratio).cwiseAbs(); // abs to add shape error
    point_edge_error = point_edge_error.array() / estimate_Quadric.scale.array();

    _error = 1.0 * point_edge_error;
}

void UnaryLocalPoint::computeError()
{
    // transform the point to local object frame
    const VertexSBAPointXYZ *pointVertex = dynamic_cast<const VertexSBAPointXYZ *>(_vertices[0]); // point position
    Vector3d local_pt = pointVertex->estimate().cwiseAbs();                                       // make it positive.
    Vector3d point_edge_error;

    // if point is within the Quadric, point_edge_error=0, otherwise penalty how far it is outside Quadric
    for (int i = 0; i < 3; i++)
    {
        if (local_pt(i) < objectscale(i))
            point_edge_error(i) = 0;
        else if (local_pt(i) < (max_outside_margin_ratio + 1) * objectscale(i))
            point_edge_error(i) = local_pt(i) - objectscale(i);
        else
            point_edge_error(i) = max_outside_margin_ratio * objectscale(i); // if points two far, give a constant error, don't optimize.
    }

    _error = point_edge_error.array() / objectscale.array();
}

} // namespace g2o
