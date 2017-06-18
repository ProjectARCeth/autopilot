#include "arc_tools/coordinate_transform.hpp"

namespace arc_tools {

geometry_msgs::Quaternion transformQuaternionEulerMsg(const geometry_msgs::Vector3 euler){
  geometry_msgs::Quaternion quat;
  //Transformation.
  float roll = euler.y;
  float pitch = -euler.x;
  float yaw = euler.z; 
  double t0 = cos(yaw * 0.5f);
  double t1 = sin(yaw * 0.5f);
  double t2 = cos(roll * 0.5f);
  double t3 = sin(roll * 0.5f);
  double t4 = cos(pitch * 0.5f);
  double t5 = sin(pitch * 0.5f);
  quat.w = t0 * t2 * t4 + t1 * t3 * t5;
  quat.x = t0 * t3 * t4 - t1 * t2 * t5;
  quat.y = t0 * t2 * t5 + t1 * t3 * t4;
  quat.z = t1 * t2 * t4 - t0 * t3 * t5;
  return quat;
}

geometry_msgs::Quaternion eulerToQuaternion(const geometry_msgs::Vector3 euler){
  geometry_msgs::Quaternion quat;
  //Transformation.
  float yaw = euler.x;
  float pitch = euler.y;
  float roll = euler.z; 
  double t0 = cos(yaw * 0.5f);
  double t1 = sin(yaw * 0.5f);
  double t2 = cos(pitch * 0.5f);
  double t3 = sin(pitch * 0.5f);
  double t4 = cos(roll * 0.5f);
  double t5 = sin(roll * 0.5f);
  quat.w = t4 * t2 * t0 + t5 * t3 * t1;
  quat.x = t5 * t2 * t0 - t4 * t3 * t1;	//t0 * t3 * t4 - t1 * t2 * t5;
  quat.y = t4 * t3 * t0 + t5 * t2 * t1;
  quat.z = t4 * t2 * t1 - t5 * t3 * t0;
  return quat;
}

Eigen::Vector4d transformQuaternionEulerVector(const Eigen::Vector3d euler){
  Eigen::Vector4d quat;
  //Transformation.
  float roll = euler(0);
  float pitch = euler(1);
  float yaw = euler(2); 
  double t0 = cos(yaw * 0.5f);
  double t1 = sin(yaw * 0.5f);
  double t2 = cos(roll * 0.5f);
  double t3 = sin(roll * 0.5f);
  double t4 = cos(pitch * 0.5f);
  double t5 = sin(pitch * 0.5f);
  quat(3) = t0 * t2 * t4 + t1 * t3 * t5;
  quat(0) = t0 * t3 * t4 - t1 * t2 * t5;
  quat(1) = t0 * t2 * t5 + t1 * t3 * t4;
  quat(2) = t1 * t2 * t4 - t0 * t3 * t5;
  return quat;
}

Eigen::Vector3d YPRFromQuaternion(const geometry_msgs::Quaternion quat){
	//euler(0)=yaw	euler(1)=pitch	euler(2)=roll
	Eigen::Vector3d euler;
	euler(0)=atan2(2*(quat.w*quat.z+quat.x*quat.y),(1-2*(quat.y*quat.y+quat.z*quat.z)));
	if(2*(quat.w*quat.y-quat.x*quat.z)<-1) euler(1)=-M_PI/2;
	if(2*(quat.w*quat.y-quat.x*quat.z)>1) euler(1)=M_PI/2;
	else euler(1)=asin(2*(quat.w*quat.y-quat.x*quat.z));
	euler(2)=atan2(2*(quat.w*quat.x+quat.y*quat.z),(1-2*(quat.x*quat.x+quat.y*quat.y)));
	return euler;
}
geometry_msgs::Vector3 transformEulerQuaternionMsg(const Eigen::Vector4d quat){
  geometry_msgs::Vector3 euler;
  //Transformation.
	double ysqr = quat(1) * quat(1);

	// roll (x-axis rotation)
	double t0 = +2.0 * (quat(3) * quat(0) + quat(1) * quat(2));
	double t1 = +1.0 - 2.0 * (quat(0) * quat(0) + ysqr);
	euler.y = std::atan2(t0, t1);

	// pitch (y-axis rotation)
	double t2 = +2.0 * (quat(3) * quat(1) - quat(2) * quat(0));
	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;
	euler.x = -std::asin(t2);

	// yaw (z-axis rotation)
	double t3 = +2.0 * (quat(3) * quat(2) + quat(0) * quat(1));
	double t4 = +1.0 - 2.0 * (ysqr + quat(2) * quat(2));  
	euler.z = std::atan2(t3, t4);
  return euler;
}

Eigen::Vector3d transformEulerQuaternionVector(const Eigen::Vector4d quat){
  Eigen::Vector3d euler;
  //Transformation.
	double ysqr = quat(1) * quat(1);

	// roll (x-axis rotation)
	double t0 = +2.0 * (quat(3) * quat(0) + quat(1) * quat(2));
	double t1 = +1.0 - 2.0 * (quat(0) * quat(0) + ysqr);
	euler(1) = std::atan2(t0, t1);

	// pitch (y-axis rotation)
	double t2 = +2.0 * (quat(3) * quat(1) - quat(2) * quat(0));
	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;
	euler(0) = -std::asin(t2);

	// yaw (z-axis rotation)
	double t3 = +2.0 * (quat(3) * quat(2) + quat(0) * quat(1));
	double t4 = +1.0 - 2.0 * (ysqr + quat(2) * quat(2));  
	euler(2) = std::atan2(t3, t4);
  return euler;
}

Eigen::Matrix3d getRotationMatrix(const Eigen::Vector3d angles){	
  double roll = angles(0);
  double pitch = angles(1);
  double yaw = angles(2);
  //Rotation matrix.
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix<<	cos(yaw)*cos(pitch), 	cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll),	cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll) ,
			sin(yaw)*cos(pitch),	sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll),	sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll),
			-sin(pitch),		cos(pitch)*sin(roll), 						cos(pitch)*cos(roll);
  return rotation_matrix;
}

Eigen::Matrix3d getAngularVelocityTransformationMatrix(const Eigen::Vector3d angles){
  Eigen::Matrix<double,3,3> Trafomatrix;
  double roll = angles(0);
  double pitch = angles(1);
  double yaw = angles(2);
  Trafomatrix(0,0) = 1.0; Trafomatrix(0,1) = sin(roll)*tan(pitch); Trafomatrix(0,2) = cos(roll)*tan(pitch);
  Trafomatrix(1,0) = 0.0; Trafomatrix(1,1) = cos(roll); Trafomatrix(1,2) = -sin(roll);
  Trafomatrix(2,0) = 0.0; Trafomatrix(2,1) = sin(roll)/cos(pitch); Trafomatrix(2,2) = cos(roll)/cos(pitch);
  return Trafomatrix;
}

Eigen::Vector3d transformVectorMessageToEigen(const geometry_msgs::Vector3 msg){
  Eigen::Vector3d eigen_vector;
  eigen_vector(0) = msg.x;
  eigen_vector(1) = msg.y;
  eigen_vector(2) = msg.z;
  return eigen_vector;
}

Eigen::Vector3d transformPointMessageToEigen(const geometry_msgs::Point msg){
  Eigen::Vector3d eigen_vector;
  eigen_vector(0) = msg.x;
  eigen_vector(1) = msg.y;
  eigen_vector(2) = msg.z;
  return eigen_vector;
}

geometry_msgs::Point transformEigenToPointMessage(const Eigen::Vector3d msg){
  geometry_msgs::Point point;
  point.x=msg(0);
  point.y=msg(1);
  point.z=msg(2);

  return point;
}

Eigen::Vector4d transformQuatMessageToEigen(const geometry_msgs::Quaternion msg){
  Eigen::Vector4d eigen_vector;
  eigen_vector(0) = msg.x;
  eigen_vector(1) = msg.y;
  eigen_vector(2) = msg.z;
  eigen_vector(3) = msg.w;
  return eigen_vector;
} 

geometry_msgs::Quaternion transformEigenToQuatMessage(const Eigen::Vector4d msg){
  geometry_msgs::Quaternion quat;
  quat.x = msg(0);
  quat.y = msg(1);
  quat.z = msg(2);
  quat.w = msg(3);
  return quat;
}

geometry_msgs::Point globalToLocal(const geometry_msgs::Point global_koordinate, arc_msgs::State new_frame_origin){
  //Translatation
  Eigen::Vector3d glob=arc_tools::transformPointMessageToEigen(global_koordinate);
  Eigen::Vector3d stat= arc_tools::transformPointMessageToEigen(new_frame_origin.pose.pose.position);
  Eigen::Vector3d temp=glob-stat;
  //Rotation
  Eigen::Vector4d quat=transformQuatMessageToEigen(new_frame_origin.pose.pose.orientation);
  Eigen::Vector3d euler=transformEulerQuaternionVector(quat);
  Eigen::Matrix3d R=getRotationMatrix(euler);
  Eigen::Matrix3d T;
  double a=quat(3);
  double b=quat(0);
  double c=quat(1);
  double d=quat(2);

  R<<(1-2*(c*c + d*d)), 2*(b*c - a*d), 2*(b*d + a*c), 
    2*(b*c + a*d), (1-2*(d*d + b*b)), 2*(c*d - a*b), 
    2*(b*d - a*c), 2*(c*d + a*b), (1-2*(b*b + c*c));
  T=R.transpose();

  //R.transpose();
  Eigen::Vector3d local=T*temp;
  geometry_msgs::Point local_msg=transformEigenToPointMessage(local);
  geometry_msgs::Point local_msg_new_axes;
  local_msg_new_axes.x=-local_msg.y;
  local_msg_new_axes.y=local_msg.x;
  local_msg_new_axes.z=local_msg.z;
  return local_msg_new_axes;
}

geometry_msgs::Point rotationLocalToGlobal(geometry_msgs::Point local_koordinate,arc_msgs::State frame){
  Eigen::Vector3d local = arc_tools::transformPointMessageToEigen(local_koordinate);
  Eigen::Vector4d quat=transformQuatMessageToEigen(frame.pose.pose.orientation);
  Eigen::Vector3d euler=transformEulerQuaternionVector(quat);
  Eigen::Matrix3d R=getRotationMatrix(euler);
  Eigen::Matrix3d T;
  double a=quat(3);
  double b=quat(0);
  double c=quat(1);
  double d=quat(2);

  R<<(1-2*(c*c + d*d)), 2*(b*c - a*d), 2*(b*d + a*c), 
    2*(b*c + a*d), (1-2*(d*d + b*b)), 2*(c*d - a*b), 
    2*(b*d - a*c), 2*(c*d + a*b), (1-2*(b*b + c*c));
  T=R;
  Eigen::Vector3d global = R*local;
  geometry_msgs::Point global_point = transformEigenToPointMessage(global);
  return global_point;
}

arc_msgs::State generate2DState(const geometry_msgs::Pose2D pose){
  arc_msgs::State state;
  state.pose.pose.position.x=pose.x;
  state.pose.pose.position.y=pose.y;
  geometry_msgs::Vector3 euler;
  euler.x=pose.theta;
  euler.y=0;
  euler.z=0;
  geometry_msgs::Quaternion quat;
  quat=arc_tools::eulerToQuaternion(euler);
  state.pose.pose.orientation=quat;
  return state;
}

//Hamiltion quaternion multiplication. (x,y,z,w)
Eigen::Vector4d multQuaternion(Eigen::Vector4d q1,Eigen::Vector4d q2){
  Eigen::Vector3d axes1(q1(0), q1(1), q1(2));
  Eigen::Vector3d axes2(q2(0), q2(1), q2(2));
  //Hamiltonian product to get new quaternion.
  Eigen::Vector4d new_quat;
  new_quat(3) = q1(3)*q2(3) + axes1.dot(axes2);
  Eigen::Vector3d new_axes = q1(3)*axes2 + q2(3)*axes1 + axes1.cross(axes2);
  new_quat(0) = new_axes(0);
  new_quat(1) = new_axes(1);
  new_quat(2) = new_axes(2);
  new_quat = new_quat/new_quat.norm();
  return new_quat;
}

//Inverse Quaternion (iff unit quaternion!). (x,y,z,w)
Eigen::Vector4d inverseQuaternion(Eigen::Vector4d quat){
  Eigen::Vector4d inverse_quat;
  inverse_quat(0) = -quat(0);
  inverse_quat(1) = -quat(1);
  inverse_quat(2) = -quat(2);
  inverse_quat(3) = quat(3);
  return inverse_quat;
}

//Get rotation quat between to quats.
Eigen::Vector4d diffQuaternion(Eigen::Vector4d base_quat, Eigen::Vector4d target_quat){
  //Normalization.
  base_quat = base_quat/base_quat.norm();
  target_quat = target_quat/target_quat.norm();
  //Getting difference of unit quaternion.
  Eigen::Vector4d diff_quat = multQuaternion(base_quat, inverseQuaternion(target_quat));
  diff_quat.normalize();
  return diff_quat;
}

geometry_msgs::Point addPoints(geometry_msgs::Point point_1, geometry_msgs::Point point_2){
  geometry_msgs::Point point;
  point.x = point_1.x + point_2.x;
  point.y = point_1.y + point_2.y;
  point.z = point_1.z + point_2.z;
  return point;
}
}//namespace arc_tools.
