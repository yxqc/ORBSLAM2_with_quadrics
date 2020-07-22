#ifndef G2O_OBJECT_H
#define G2O_OBJECT_H

#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "matrix_utils.h"

#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/StdVector>

#include <algorithm> // std::swap
#include <iostream>
#define WIDTH 640
#define HEIGHT 480

typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 10, 1> Vector10d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 5, 1> Vector5d;
typedef Eigen::Matrix<double, 4, 1> Vector4d;

namespace g2o
{

class Quadric
{
public:
  SE3Quat pose;
  SE3Quat pose1;
  Vector3d scale; // semi-axis a,b,c

  Quadric()
  {
    pose = SE3Quat();
    scale.setZero();
  }

  Quadric(const Matrix3d &R, const Vector3d &t, const Vector3d &inputScale)
  {
    pose = SE3Quat(R, t);
    scale = inputScale;
  }

  /*
   * v = (t1,t2,t3,theta1,theta2,theta3,s1,s2,s3)
   * xyz roll pitch yaw half_scale
   * construct quadric from vector9d to SE3Quat
   */
  inline void fromMinimalVector(const Vector9d &v)
  {
    Eigen::Quaterniond posequat = zyx_euler_to_quat(v(3), v(4), v(5));
    pose = SE3Quat(posequat, v.head<3>());
    scale = v.tail<3>();
  }

   inline Vector3d point_boundary_error(const Vector3d &world_point, const double max_outside_margin_ratio, double point_scale = 1) const
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

  // dual quadric: 4*4 symmetric matrix with 10DoF
  inline void fromVector10d(const Vector10d &v)
  {
    Eigen::Matrix4d dual_quadric, raw_quadric;
    Eigen::Vector3d rotation;
    Eigen::Vector3d shape;
    Eigen::Vector3d translation;

    dual_quadric << v(0, 0), v(1, 0), v(2, 0), v(3, 0),
        v(1, 0), v(4, 0), v(5, 0), v(6, 0),
        v(2, 0), v(5, 0), v(7, 0), v(8, 0),
        v(3, 0), v(6, 0), v(8, 0), v(9, 0);
    raw_quadric = dual_quadric.inverse() * cbrt(dual_quadric.determinant());

    //开始重建约束 rebuild constrain
    Eigen::Matrix3d quadric_33;
    quadric_33 = raw_quadric.block(0, 0, 3, 3);
    
    Eigen::Vector3d quadric_t;
    quadric_t = raw_quadric.block(0,3,3,1);

    //quadric33 特征矩阵为旋转参数
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(quadric_33);
    Eigen::Matrix3d eigen_vectors = eigen_solver.eigenvectors();
    rotation = eigen_vectors.eulerAngles(2, 1, 0);

    //计算shape参数
    double det = raw_quadric.determinant() / quadric_33.determinant();
    Eigen::Vector3d eigen_values;
    eigen_values = eigen_solver.eigenvalues();
    Eigen::Vector3d eigen_values_inverse;
    eigen_values_inverse = eigen_values.array().inverse();
    shape << (((-det) * eigen_values_inverse).array().abs()).array().sqrt();

    //计算平移参数
   translation << v(3, 0) / v(9, 0), v(6, 0) / v(9, 0), v(8, 0) / v(9, 0);
    //translation<<quadric_33*quadric_t;
    


    Eigen::Quaterniond posequat = zyx_euler_to_quat(
        rotation(0, 0), rotation(1, 0), rotation(2, 0)); // may be xyz_euler
    Eigen::Vector3d test = posequat*translation;
    std::cout<<"Test"<<test<<std::endl;
    ///pose = SE3Quat(posequat,test.head<3>());
    pose = SE3Quat(posequat, translation.head<3>());
    scale = shape.head<3>();
  }

  inline const Vector3d &translation() const { return pose.translation(); }
  inline void setTranslation(const Vector3d &t_) { pose.setTranslation(t_); }
  inline void setRotation(const Quaterniond &r_) { pose.setRotation(r_); }
  inline void setRotation(const Matrix3d &R_) { pose.setRotation(Quaterniond(R_)); }
  inline void setScale(const Vector3d &scale_) { scale = scale_; }

  // apply update to current quadric, exponential map
  Quadric exp_update(const Vector9d &update)
  {
    Quadric res;
    res.pose = this->pose * SE3Quat::exp(update.head<6>());
    res.scale = this->scale + update.tail<3>();
    return res;
  }

  // transform a local Quadric to global Quadric  Twc is camera pose. from camera
  // to world
  Quadric transform_from(const SE3Quat &Twc) const
  {
    Quadric res;
    res.pose = Twc * this->pose;
    res.scale = this->scale;
    return res;
  }

  // transform a global Quadric to local Quadric  Twc is camera pose. from camera to world
  Quadric transform_to(const SE3Quat &Twc) const
  {
    Quadric res;
    res.pose = Twc.inverse() * this->pose;
    res.scale = this->scale;
    return res;
  }

  // xyz roll pitch yaw half_scale
  inline Vector9d toMinimalVector() const
  {
    Vector9d v;
    v.head<3>() = pose.translation();
    quat_to_euler_zyx(pose.rotation(), v(3, 0), v(4, 0), v(5, 0));
    v.tail<3>() = scale;
    return v;
  }

  //return xyz quaternion, half_scale(three semi-axis abc)
  /*inline Vector10d toVector() const
  {
    //seems no necessary to do
  }*/

  Matrix4d toSymMat() const
  {
    Matrix4d res;
    Matrix4d centreAtOrigin;
    centreAtOrigin = Eigen::Matrix4d::Identity();
    centreAtOrigin(0, 0) = pow(scale(0), 2);
    centreAtOrigin(1, 1) = pow(scale(1), 2);
    centreAtOrigin(2, 2) = pow(scale(2), 2);
    centreAtOrigin(3, 3) = -1;
    Matrix4d Z;
     Z =pose.to_homogeneous_matrix();
     //Z = pose1.to_homogeneous_matrix();
    res = Z * centreAtOrigin * Z.transpose();

    return res;
  }

  //transform to 10-parameters representation
  inline Vector10d toVector10d() const
  {
    Matrix4d Q = this->toSymMat();
    Vector10d v;
    v << Q(0, 0), Q(0, 1), Q(0, 2), Q(0, 3), Q(1, 1), Q(1, 2), Q(1, 3), Q(2, 2), Q(2.3), Q(3, 3);
    return v;
  }

  // get rectangles after projection  [topleft, bottomright] inverse() to get adjugate()
  Matrix3d toConic(const SE3Quat &campose_cw, const Matrix3d &calib) const
  {
    Eigen::Matrix<double, 3, 4> P =
        calib * campose_cw.to_homogeneous_matrix().block(
                    0, 0, 3, 4); // Todo:BUG!! maybe campose_cw
    Matrix4d symMat = this->toSymMat();
    Matrix3d conic = (P * symMat * P.transpose());
    return conic.inverse();
  }

  //return x_min y_min x_max y_max

  Vector4d projectOntoImageRect(const SE3Quat &campose_cw,
                                const Matrix3d &Kalib) const
  {
    //    std::cout << "projectOntoImageRect" << std::endl;
    Matrix3d conic = this->toConic(campose_cw, Kalib);
    double delt = conic(1, 1) * ((conic(0, 0) * conic(1, 1) - pow(conic(0, 1), 2)) * conic(2, 2) + conic(0, 1) * conic(1, 2) * conic(0, 2) * 2 - conic(1, 1) * pow(conic(0, 2), 2) - conic(0, 0) * pow(conic(1, 2), 2));
    assert(conic.determinant() < 0);
    /* if(delt< 0){
      Vector6d c;
     c << conic(0, 0), conic(0, 1) * 2, conic(1, 1), conic(0, 2) * 2,
        conic(1, 2) * 2, conic(2, 2);
     std::cout << "c: " << c << std::endl;
    double centerX = 
        (2 * c(2) * c(3) - c(1) * c(4)) / (c(1) * c(1) - 4 * c(0) * c(2));
     double centerY =
          (2 * c(0) * c(4) - c(1) * c(3)) / (c(1) * c(1) - 4 * c(0) * c(2));

     double theta = abs(
        atan((c(2) - c(0) - sqrt((c(0) - c(2)) * (c(0) - c(2)) + c(1) *
          c(1))) /
              c(1)));
     double radiiX = sqrt(2*(c(0)*pow(c(4),2)+c(2)*pow(c(3),2)-c(1)*c(3)*c(4)+(pow(c(1),2)-4*c(0)*c(2))*c(5))*(c(0)+c(2)+sqrt(pow(c(0)-c(2),2)+pow(c(1),2))))/(pow(c(1),2)-4*c(0)*c(2));
 
     double radiiY = sqrt(2*(c(0)*pow(c(4),2)+c(2)*pow(c(3),2)-c(1)*c(3)*c(4)+(pow(c(1),2)-4*c(0)*c(2))*c(5))*(c(0)+c(2)-sqrt(pow(c(0)-c(2),2)+pow(c(1),2))))/(pow(c(1),2)-4*c(0)*c(2));
     
    double a1=radiiX*cos(theta);
    double b1 =radiiY*sin(theta);
    double c1 =radiiX*sin(theta);
    double d1 =radiiY*cos(theta);
    double width = sqrt(pow(a1,2)+pow(b1,2))*2;
    double height = sqrt(pow(c1,2)+pow(d1,2))*2;
    double X = centerX-width*0.5;
    double Y = centerY-height*0.5;
   




     Vector2d bottomright;  // x y
     Vector2d topleft;
    topleft(0) = centerX-width*0.5;

    topleft(1) = centerY-height*0.5;
    bottomright(0) = topleft(0)+width;
     bottomright(1) = topleft(1)+height;
   // Todo:conic at boundary
     std::cout << "projectOntoImageRect end"
             << Vector4d(topleft(0), topleft(1), bottomright(0),
                          bottomright(1))
              << std::endl;
   
  return Vector4d(topleft(0), topleft(1), bottomright(0), bottomright(1));
    }
  else  return Vector4d::Identity();
   }
  */
    if (delt >= 0)
      std::cout << "it's an imaginary or a point ellipse" << std::endl;

    Vector6d c;
    c << conic(0, 0), conic(0, 1) * 2, conic(1, 1), conic(0, 2) * 2,
        conic(1, 2) * 2, conic(2, 2);
    std::cout << "Conic" << c << std::endl;

    Vector2d y, x;
    y(0) = (4 * c(4) * c(0) - 2 * c(1) * c(3) +
            sqrt(pow(2 * c(1) * c(3) - 4 * c(0) * c(4), 2) -
                 4 * (pow(c(1), 2) - 4 * c(0) * c(2)) *
                     (pow(c(3), 2) - 4 * c(0) * c(5)))) /
           (2 * (pow(c(1), 2) - 4 * c(2) * c(0)));

    y(1) = (4 * c(4) * c(0) - 2 * c(1) * c(3) -
            sqrt(pow(2 * c(1) * c(3) - 4 * c(0) * c(4), 2) -
                 4 * (pow(c(1), 2) - 4 * c(0) * c(2)) *
                     (pow(c(3), 2) - 4 * c(0) * c(5)))) /
           (2 * (pow(c(1), 2) - 4 * c(2) * c(0)));

    x(0) = (4 * c(3) * c(2) - 2 * c(1) * c(4) +
            sqrt(pow(2 * c(1) * c(4) - 4 * c(2) * c(3), 2) -
                 4 * (pow(c(1), 2) - 4 * c(0) * c(2)) *
                     (pow(c(4), 2) - 4 * c(2) * c(5)))) /
           (2 * (pow(c(1), 2) - 4 * c(2) * c(0)));

    x(1) = (4 * c(3) * c(2) - 2 * c(1) * c(4) -
            sqrt(pow(2 * c(1) * c(4) - 4 * c(2) * c(3), 2) -
                 4 * (pow(c(1), 2) - 4 * c(0) * c(2)) *
                     (pow(c(4), 2) - 4 * c(2) * c(5)))) /
           (2 * (pow(c(1), 2) - 4 * c(2) * c(0)));
    Vector2d bottomright; // x y
    Vector2d topleft;
    bottomright(0) = x.maxCoeff();
    bottomright(1) = y.maxCoeff();
    topleft(0) = x.minCoeff();
    topleft(1) = y.minCoeff();
    std::cout<<"B^2-4AC"<<c(1)*c(1)-4*c(0)*c(2)<<std::endl;
    double centerX = 
        (2 * c(2) * c(3) - c(1) * c(4)) / (c(1) * c(1) - 4 * c(0) * c(2));
    double centerY =
          (2 * c(0) * c(4) - c(1) * c(3)) / (c(1) * c(1) - 4 * c(0) * c(2));

    double theta ;
    if(c(0)<c(2)&&c(1) == 0){
         theta = 0;
       }
    if(c(0)>c(2)&&c(1) ==0){
           theta = 90;
       }
    if(c(1)!=0){
      std::cout<<"c1"<<std::endl; 
      theta= abs(
        atan((c(2) - c(0) - sqrt((c(0) - c(2)) * (c(0) - c(2)) + c(1) *
          c(1))) /
              c(1)));
      }
    std::cout<<"theta"<<theta<<std::endl;
    double radiiX1 = (-(sqrt((2*(c(0)*pow(c(4),2)+c(2)*pow(c(3),2)-c(1)*c(3)*c(4)+(pow(c(1),2)-4*c(0)*c(2))*c(5)))*(c(0)+c(2)+sqrt(pow(c(0)-c(2),2)+pow(c(1),2))))))/(pow(c(1),2)-4*c(0)*c(2));
 
    double radiiY1 = (-(sqrt((2*(c(0)*pow(c(4),2)+c(2)*pow(c(3),2)-c(1)*c(3)*c(4)+(pow(c(1),2)-4*c(0)*c(2))*c(5)))*(c(0)+c(2)-sqrt(pow(c(0)-c(2),2)+pow(c(1),2))))))/(pow(c(1),2)-4*c(0)*c(2));
    std::cout<<"raddd"<<radiiX1 <<" "<<radiiY1<<std::endl;
    double radiiX = abs(sqrt((2*(c(0)*pow(c(4),2)+c(2)*pow(c(3),2)-c(1)*c(3)*c(4)+(pow(c(1),2)-4*c(0)*c(2))*c(5)))*(c(0)+c(2)+sqrt(pow(c(0)-c(2),2)+pow(c(1),2))))/(pow(c(1),2)-4*c(0)*c(2)));
 
    double radiiY = abs(sqrt((2*(c(0)*pow(c(4),2)+c(2)*pow(c(3),2)-c(1)*c(3)*c(4)+(pow(c(1),2)-4*c(0)*c(2))*c(5)))*(c(0)+c(2)-sqrt(pow(c(0)-c(2),2)+pow(c(1),2))))/(pow(c(1),2)-4*c(0)*c(2)));
    double X,Y,height,width = 0;
    if(radiiX > 0 &&radiiY > 0){
 
      double a1=radiiX*cos(theta);
      double b1 =radiiY*sin(theta);
      double c1 =radiiX*sin(theta);
      double d1 =radiiY*cos(theta);
      width = sqrt(pow(a1,2)+pow(b1,2))*2;
      height = sqrt(pow(c1,2)+pow(d1,2))*2;
      X = centerX-width*0.5;
      Y = centerY-height*0.5;
    }
    else
     {
      X = Y = height = width = radiiY = radiiX = 0;
    }
    std::cout<<"center"<<centerX<<" "<<centerY<<std::endl; 
    std::cout<<"sqrt"<<sqrt(2*(c(0)*pow(c(4),2)+c(2)*pow(c(3),2)-c(1)*c(3)*c(4)+(pow(c(1),2)-4*c(0)*c(2))*c(5))*(c(0)+c(2)-sqrt(pow(c(0)-c(2),2)+pow(c(1),2))))<<std::endl;
    std::cout<<"SQRT"<<sqrt(pow(c(0)-c(2),2)+pow(c(1),2))<<std::endl;
    std::cout<<"sss"<<abs(c(0)*pow(c(4),2)+c(2)*pow(c(3),2)-c(1)*c(3)*c(4)+(pow(c(1),2)-4*c(0)*c(2))*c(5))<<std::endl;
   std::cout<<"radix"<<radiiX<<" "<<radiiY<<" "<<std::endl;
    std::cout<<"test"<<X<<" " <<Y<<" "<<X+width<<" "<<Y+height<<std::endl;//added by yxqc one method to compute conic_bbox;
    std::cout<<"test_conic"<<topleft(0) <<" " << topleft(1) <<" "<<bottomright(0)<<" "<<bottomright(1)<<std::endl;

    return Vector4d(X,Y,X+width,Y+height);
    //return Vector4d(topleft(0), topleft(1), bottomright(0), bottomright(1));
  }

  //return x y w h
  Vector4d projectOntoImageBbox(const SE3Quat &campose_cw,
                                const Matrix3d &Kalib) const
  {
    Vector4d rect_project = projectOntoImageRect(
        campose_cw, Kalib); // top_left, bottom_right  x1 y1 x2 y2
    Vector2d rect_center =
        (rect_project.tail<2>() + rect_project.head<2>()) / 2;
    Vector2d widthheight = rect_project.tail<2>() - rect_project.head<2>();

    return Vector4d(rect_center(0), rect_center(1), widthheight(0),
                    widthheight(1));
  }
};

inline SE3Quat exptwist_norollpitch(const Vector6d &update)
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


class VertexQuadric : public BaseVertex<9, Quadric> // NOTE  this vertex stores
                                                    // object pose to world
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  VertexQuadric()
  {
    fixedscale.setZero();
    whether_fixrollpitch = false;
    whether_fixrotation = false;
    whether_fixheight = false;
   }

  virtual void setToOriginImpl() { _estimate = Quadric(); }

  virtual void oplusImpl(const double *update_)
  {
    Eigen::Map<const Vector9d> update(update_);
    setEstimate(_estimate.exp_update(update));
  }

  virtual bool read(std::istream &is)
  {
    Vector9d est;
    for (int i = 0; i < 9; i++)
      is >> est[i];
    Quadric oneQuadric;
    oneQuadric.fromMinimalVector(est);
    setEstimate(oneQuadric);
    return true;
  }

  virtual bool write(std::ostream &os) const
  {
    Vector9d lv = _estimate.toMinimalVector();
    for (int i = 0; i < lv.rows(); i++)
    {
      os << lv[i] << " ";
    }
    return os.good();
  }
   // some mode parameters. a more efficient way is to create separate vertex
    Vector3d fixedscale;	   // if want to fix scale, set it to be true value
    bool whether_fixrollpitch; // for ground object, only update yaw  
    bool whether_fixrotation;  // don't update any rotation  
    bool whether_fixheight;	// object height is fixed
  
};

class VertexQuadricFixScale : public BaseVertex<6, Quadric> // less variables. should be faster  fixed scale should be set
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	VertexQuadricFixScale()
	{
		fixedscale.setZero();
		whether_fixrollpitch = false;
		whether_fixrotation = false;
		whether_fixheight = false;
	};

	virtual void setToOriginImpl()
	{
		_estimate = Quadric();
		if (fixedscale(0) > 0)
			_estimate.scale = fixedscale;
	}

	virtual void oplusImpl(const double *update_)
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

	virtual bool read(std::istream &is) { return true; };
	virtual bool write(std::ostream &os) const { return os.good(); };

	Vector3d fixedscale;
	bool whether_fixrollpitch;
	bool whether_fixrotation;
	bool whether_fixheight;
};
// camera -object 2D projection error, rectangle difference, could also change
// to iou

class EdgeSE3QuadricProj
    : public BaseBinaryEdge<4, Vector4d, VertexSE3Expmap, VertexQuadric>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  int cout = 0;

  EdgeSE3QuadricProj(){};

  virtual bool read(std::istream &is) { return true; };

  virtual bool write(std::ostream &os) const { return os.good(); };

  void computeError()
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
  Matrix3d calib;
};

// camera -fixscale_object 2D projection error, rectangle, could also change to iou
class EdgeSE3QuadricFixScaleProj : public BaseBinaryEdge<4, Vector4d, VertexSE3Expmap, VertexQuadricFixScale>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	EdgeSE3QuadricFixScaleProj(){};

	virtual bool read(std::istream &is) { return true; };
	virtual bool write(std::ostream &os) const { return os.good(); };

	void computeError()
{
    const VertexSE3Expmap *SE3Vertex = static_cast<const VertexSE3Expmap *>(
        _vertices[0]); //  world to camera pose
    const VertexQuadricFixScale *quadricVertex = static_cast<const VertexQuadricFixScale *>(
        _vertices[1]); //  object pose to world

    SE3Quat cam_pose_Tcw = SE3Vertex->estimate();
    Quadric global_quadric = quadricVertex->estimate();

    Vector4d rect_project = global_quadric.projectOntoImageBbox(
        cam_pose_Tcw, calib); // Attention：center, width, height

    _error = (rect_project - _measurement).array().pow(2);
  }

	double get_error_norm();
	Matrix3d calib;
};


// object point surface error, both will optimize
class EdgePointQuadric : public BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexQuadric>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	EdgePointQuadric(){};

	virtual bool read(std::istream &is) { return true; };
	virtual bool write(std::ostream &os) const { return os.good(); };

	void computeError()
{};

	double max_outside_margin_ratio; // truncate the error if point is too far from object
};

// object point surface error, both will optimize.   object has fixed size.
class EdgePointQuadricFixScale : public BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexQuadricFixScale>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	EdgePointQuadricFixScale(){};

	virtual bool read(std::istream &is) { return true; };
	virtual bool write(std::ostream &os) const { return os.good(); };

        void computeError()
       {};
   double max_outside_margin_ratio; // truncate the error if point is too far from object
   
};


// one object connected with all fixed points. only optimize object.  want object to contain points
class EdgePointQuadricOnlyObject : public BaseUnaryEdge<3, Vector3d, VertexQuadric>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	EdgePointQuadricOnlyObject()
	{
		print_details = false;
		prior_object_half_size.setZero();
	};

	virtual bool read(std::istream &is) { return true; };
	virtual bool write(std::ostream &os) const { return os.good(); };

	bool print_details;

	void computeError()
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
    if (prior_object_half_size(0) > 0)                // if prior shape is being set, such as KITTI, then give large weight for shape error
    {
        prior_weight = 50.0;
        prior_shape_error = ((estimate_Quadric.scale - prior_object_half_size).array() / prior_object_half_size.array()).cwiseAbs();
    }

    _error = 1.0 * point_edge_error + prior_weight * prior_shape_error;
  }

	Vector3d computeError_debug();

	std::vector<Vector3d> object_points; // all the fixed points.
	double max_outside_margin_ratio;	 // truncate the error if point is too far from object
	Vector3d prior_object_half_size;	 // give a prior object size, otherwise a huge Quadric can always contain all points
};

// one object connected with all fixed points   similar as above
class EdgePointQuadricOnlyObjectFixScale : public BaseUnaryEdge<3, Vector3d, VertexQuadricFixScale>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	EdgePointQuadricOnlyObjectFixScale()
	{
		print_details = false;
		prior_object_half_size.setZero();
	};

	virtual bool read(std::istream &is) { return true; };
	virtual bool write(std::ostream &os) const { return os.good(); };

	bool print_details;

	void computeError()
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
	Vector3d computeError_debug();

	std::vector<Vector3d> object_points;
	double max_outside_margin_ratio;
	Vector3d prior_object_half_size;
};



} // namespace g2o

#endif
