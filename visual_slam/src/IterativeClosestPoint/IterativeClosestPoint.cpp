
void visual_slam::IterativeClosestPoint::calculateError(visual_slam::PointCloudT::ConstPtr, visual_slam::PointCloudT::ConstPtr, double &)
{

}
double visual_slam::IterativeClosestPoint::errorFunction(const Eigen::Vector4f& x1,
                      const Eigen::Vector4f& x2,
                      const Eigen::Matrix4d& transformation)
{
  //FIXME: Take from paramter_server or cam info
  static const double cam_angle_x = 58.0/180.0*M_PI;/*{{{*/
  static const double cam_angle_y = 45.0/180.0*M_PI;
  static const double cam_resol_x = 640;
  static const double cam_resol_y = 480;
  static const double raster_stddev_x = 3*tan(cam_angle_x/cam_resol_x);  //5pix stddev in x
  static const double raster_stddev_y = 3*tan(cam_angle_y/cam_resol_y);  //5pix stddev in y
  static const double raster_cov_x = raster_stddev_x * raster_stddev_x;
  static const double raster_cov_y = raster_stddev_y * raster_stddev_y;/*}}}*/
  static const bool use_error_shortcut = true;//ParameterServer::instance()->get<bool>("use_error_shortcut");

  bool nan1 = std::isnan(x1(2));
  bool nan2 = std::isnan(x2(2));
  if(nan1||nan2){
    //TODO: Handle Features with NaN, by reporting the reprojection error
    return std::numeric_limits<double>::max();
  }
  Eigen::Vector4d x_1 = x1.cast<double>();
  Eigen::Vector4d x_2 = x2.cast<double>();

  Eigen::Matrix4d tf_12 = transformation;
  Eigen::Vector3d mu_1 = x_1.head<3>();
  Eigen::Vector3d mu_2 = x_2.head<3>();
  Eigen::Vector3d mu_1_in_frame_2 = (tf_12 * x_1).head<3>(); // μ₁⁽²⁾  = T₁₂ μ₁⁽¹⁾
  //New Shortcut to determine clear outliers
  if(use_error_shortcut)
  {
    double delta_sq_norm = (mu_1_in_frame_2 - mu_2).squaredNorm();
    double sigma_max_1 = std::max(raster_cov_x, depth_covariance(mu_1(2)));//Assuming raster_cov_x and _y to be approx. equal
    double sigma_max_2 = std::max(raster_cov_x, depth_covariance(mu_2(2)));//Assuming raster_cov_x and _y to be approx. equal
    if(delta_sq_norm > 2.0 * (sigma_max_1+sigma_max_2)) //FIXME: Factor 3 for mahal dist should be gotten from caller
    {
      return std::numeric_limits<double>::max();
    }
  }

  Eigen::Matrix3d rotation_mat = tf_12.block(0,0,3,3);

  //Point 1
  Eigen::Matrix3d cov1 = Eigen::Matrix3d::Zero();
  cov1(0,0) = raster_cov_x * mu_1(2); //how big is 1px std dev in meter, depends on depth
  cov1(1,1) = raster_cov_y * mu_1(2); //how big is 1px std dev in meter, depends on depth
  cov1(2,2) = depth_covariance(mu_1(2));

  //Point2
  Eigen::Matrix3d cov2 = Eigen::Matrix3d::Zero();
  cov2(0,0) = raster_cov_x* mu_2(2); //how big is 1px std dev in meter, depends on depth
  cov2(1,1) = raster_cov_y* mu_2(2); //how big is 1px std dev in meter, depends on depth
  cov2(2,2) = depth_covariance(mu_2(2));

  Eigen::Matrix3d cov1_in_frame_2 = rotation_mat.transpose() * cov1 * rotation_mat;//Works since the cov is diagonal => Eig-Vec-Matrix is Identity

  // Δμ⁽²⁾ =  μ₁⁽²⁾ - μ₂⁽²⁾
  Eigen::Vector3d delta_mu_in_frame_2 = mu_1_in_frame_2 - mu_2;
  if(std::isnan(delta_mu_in_frame_2(2))){
    ROS_ERROR("Unexpected NaN");
    return std::numeric_limits<double>::max();
  }
  // Σc = (Σ₁ + Σ₂)
  Eigen::Matrix3d cov_mat_sum_in_frame_2 = cov1_in_frame_2 + cov2;
  //ΔμT Σc⁻¹Δμ
  //double sqrd_mahalanobis_distance = delta_mu_in_frame_2.transpose() * cov_mat_sum_in_frame_2.inverse() * delta_mu_in_frame_2;
  double sqrd_mahalanobis_distance = delta_mu_in_frame_2.transpose() *cov_mat_sum_in_frame_2.llt().solve(delta_mu_in_frame_2);

  if(!(sqrd_mahalanobis_distance >= 0.0))
  {
    return std::numeric_limits<double>::max();
  }
  return sqrd_mahalanobis_distance;
}


