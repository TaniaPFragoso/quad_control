#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <eigen3/Eigen/Dense>

Eigen::Vector3f omega_punto(0,0,0);
Eigen::Vector3f omega(0,0,0);

Eigen::Vector3f posicion_angular(0,0,0);
Eigen::Vector3f vel_angular_inercial(0,0,0);
Eigen::Vector3f vel_lineal_inercial(0,0,0);
Eigen::Vector3f posicion_lineal(-5,0,0);

Eigen::Vector3f torques;
Eigen::Vector3f fuerzas(0,0,0);
Eigen::Vector3f v_punto(0,0,0);
Eigen::Vector3f v(0,0,0);

float step = .01;
float thrust;
float m = 2;
float gravedad = 9.81;
float Jxx = .0411;
float Jyy = .0478;
float Jzz = .0599;

Eigen::Vector3f e3(0,0,1);

//Funciones
Eigen::Matrix3f Rotacion_matrix(float psi, float phi, float theta) 
{

    Eigen::Matrix3f matriz_rotacion;
    matriz_rotacion << cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta)-cos(phi)*sin(psi), sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta),
                      cos(theta)*sin(psi), cos(psi)*cos(phi) + sin(psi)*sin(phi)*sin(theta), cos(phi)*sin(psi)*sin(theta)-cos(psi)*sin(phi),
                      -sin(theta), cos(theta)*sin(phi), cos(phi)*cos(theta);
    return matriz_rotacion;
}

Eigen::Matrix3f R2(float phi, float theta)
{

    Eigen::Matrix3f matrizR2;
    matrizR2 << 1, sin(phi)*tan(theta), cos(phi)*tan(theta),
                0, cos(phi), -sin(phi),
                0, sin(phi)/cos(theta), cos(phi)/cos(theta);
    return matrizR2;
}

// Corrección aquí: Cambiar la definición de sk para tomar un solo vector
Eigen::Matrix3f sk(Eigen::Vector3f omega)
{
    Eigen::Matrix3f matrixSK;
    matrixSK << 0, -omega.z(), omega.y(),
                omega.z(), 0, -omega.x(),
                -omega.y(), omega.x(), 0;
    return matrixSK;
}

void ThrustCallback(const geometry_msgs::Twist::ConstPtr& thr)
{
    thrust = thr->linear.x;
}

void TorqueRollCallback(const geometry_msgs::Twist::ConstPtr& tr)
{
    torques(0) = tr->angular.z;
}

void TorquePitchCallback(const geometry_msgs::Twist::ConstPtr& tp)
{
    torques(1) = tp->angular.z;
}

void TorqueYawCallback(const geometry_msgs::Twist::ConstPtr& ty)
{
    torques(2) = ty->angular.z;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "dinamica");
    ros::NodeHandle nh;

    ros::Subscriber thrust_sub = nh.subscribe("/thrust_rolldes_pitchdes", 10, ThrustCallback);
    ros::Subscriber torqueR_sub = nh.subscribe("/torque_roll", 10, TorqueRollCallback);
    ros::Subscriber torqueP_sub = nh.subscribe("/torque_pitch", 10, TorquePitchCallback);
    ros::Subscriber torqueY_sub = nh.subscribe("/torque_yaw", 10, TorqueYawCallback);

    ros::Publisher posicion_pub = nh.advertise<geometry_msgs::Twist>("/posicion_linear_angular", 10);
    ros::Publisher velocidad_pub = nh.advertise<geometry_msgs::Twist>("/velocidad_linear_angular", 10);

    ros::Publisher gazebo_position_pub = nh.advertise<geometry_msgs::Vector3>("/quad_position",100);
    ros::Publisher gazebo_attitude_pub = nh.advertise<geometry_msgs::Vector3>("/quad_attitude",100);

    // Relacion entre nodos
    geometry_msgs::Twist posicion_var;
    geometry_msgs::Twist velocidad_var;

    //Relacion entre gazebo
    geometry_msgs::Vector3 position;
    geometry_msgs::Vector3 attitude;

    Eigen::Matrix3f Jo;
    Jo << Jxx, 0, 0,
          0, Jyy, 0,
          0, 0, Jzz;

    ros::Rate loop_rate(100);
    
    while(ros::ok())
    {
        //Dinámica Angular
        omega_punto = Jo.inverse() * (torques - sk(omega) * Jo * omega);
        omega = omega + step * omega_punto;
        vel_angular_inercial = R2(posicion_angular(0), posicion_angular(1)) * omega;
        for(int i = 0; i < 3; i++)
        {
            posicion_angular(i) += step * vel_angular_inercial(i);
        }
        
        //Dinámica Lineal
        fuerzas = thrust * e3 + Rotacion_matrix(posicion_angular(2), posicion_angular(0), posicion_angular(1)).inverse()*(m * gravedad * e3); //z hacia abajo
        v_punto = (fuerzas / m) - sk(omega) * v;
        v = v + step * v_punto;

        vel_lineal_inercial = Rotacion_matrix(posicion_angular(2), posicion_angular(0), posicion_angular(1)) * v;
        posicion_lineal = posicion_lineal + step * vel_lineal_inercial;

        posicion_var.linear.x = posicion_lineal(0); //x
        posicion_var.linear.y = posicion_lineal(1); //y
        posicion_var.linear.z = posicion_lineal(2); //z

        posicion_var.angular.x = posicion_angular(0); //roll
        posicion_var.angular.y = posicion_angular(1); //pitch
        posicion_var.angular.z = posicion_angular(2); //yaw

        velocidad_var.linear.x = vel_lineal_inercial(0); //vel_x
        velocidad_var.linear.y = vel_lineal_inercial(1); //vel_y
        velocidad_var.linear.z = vel_lineal_inercial(2); //vel_z

        velocidad_var.angular.x = vel_angular_inercial(0); //vel_roll
        velocidad_var.angular.y = vel_angular_inercial(1); //vel_pitch
        velocidad_var.angular.z = vel_angular_inercial(2); //vel_yaw

        velocidad_pub.publish(velocidad_var);
        posicion_pub.publish(posicion_var);

        //Guardado para enviar a gazebo
        position.x = posicion_lineal(0); //x
        position.y = posicion_lineal(1); //y
        position.z = posicion_lineal(2); //z

        attitude.x = posicion_angular(0); //roll
        attitude.y = posicion_angular(1); //pitch
        attitude.z = posicion_angular(2); //yaw

        gazebo_position_pub.publish(position);
        gazebo_attitude_pub.publish(attitude);


        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}