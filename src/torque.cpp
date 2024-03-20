#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

// Declaración de variables globales para almacenar los valores de roll y pitch deseados
float pitch_des = 0;
float roll_des = 0;
float yaw_des =0;

// Declaración de constantes para los momentos de inercia
const float Jxx = 0.0411;
const float Jyy = 0.0478;
const float Jzz = 0.0599;

// Declaración de constantes de ganancia para los controladores PD
const float kproll = 500;
const float kdroll = 20;
const float kppitch = 500;
const float kdpitch = 20;
const float kpyaw = 500;
const float kdyaw = 20;

// Declaración de variables globales para almacenar los errores y sus derivadas
float roll = 0, pitch = 0, yaw = 0;
float vel_roll = 0, vel_pitch = 0, vel_yaw = 0;
float vel_des_roll = 0, vel_des_pitch = 0, vel_des_yaw = 0;

// Callback para manejar los mensajes recibidos en el tópico "velocidad_pub"
void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    // Obtener los valores de roll y pitch deseados del mensaje recibido
    roll_des = msg->linear.y;
    pitch_des = msg->linear.z;
}

void pos_desCallback(const geometry_msgs::Twist::ConstPtr& pou) {
    // Obtener los valores de yaw deseados del mensaje recibido
    yaw_des = pou->angular.z;

}

void posCallback(const geometry_msgs::Twist::ConstPtr& pos) //Recibidor de posiciones lineales. Dinamica
{

    roll=pos-> angular.x;
    pitch=pos -> angular.y;
    yaw= pos-> angular.z;
    
}

void velCallback(const geometry_msgs::Twist::ConstPtr& vel) //Recibidor de velocidad lineales. Dinamica
{
    vel_roll = vel-> angular.x;
    vel_pitch = vel -> angular.y;
    vel_yaw = vel -> angular.z;
    
}

void vel_desCallback(const geometry_msgs::Twist::ConstPtr& veld) //Recibidor de velocidades deseadas. Referencia
{

    vel_des_yaw = veld -> angular.z;
}


int main(int argc, char **argv) {

    // Inicializar el nodo ROS
    ros::init(argc, argv, "torque");
    ros::NodeHandle nh;

    // Suscribirse al tópico "velocidad_pub" para obtener los valores 
    ros::Subscriber sub = nh.subscribe("/thrust_rolldes_pitchdes", 1, velocityCallback);
    ros::Subscriber posicion_deseada_sub = nh.subscribe("/pos_des", 10, &pos_desCallback);
    ros::Subscriber posicion_sub = nh.subscribe("/posicion_linear_angular", 10, &posCallback);//Topico de velocidades deseadas (viene de dinamica)
    ros::Subscriber velocidades_sub = nh.subscribe("/velocidad_linear_angular", 10, &velCallback);//Topico de velocidades deseadas (viene de dinamica)
    ros::Subscriber velocidades_deseadas_sub = nh.subscribe("/vel_des", 10, &vel_desCallback);//Topico de velocidades deseadas (viene de referencias)
 



    // Declarar los publishers para los torques de yaw, pitch y roll
    ros::Publisher pub_torque_roll = nh.advertise<geometry_msgs::Twist>("/torque_roll", 10);
    ros::Publisher pub_torque_pitch = nh.advertise<geometry_msgs::Twist>("/torque_pitch", 10);
    ros::Publisher pub_torque_yaw = nh.advertise<geometry_msgs::Twist>("/torque_yaw", 10);

    // Nos pide la frecuencia de 100HZ
    ros::Rate rate(100); 

    while (ros::ok()) 
    {
        // Calcular los errores
        float error_roll = roll - roll_des;
        float error_pitch = pitch - pitch_des;
        float error_yaw = yaw - yaw_des;

        // Calcular las derivadas de los errores
        float error_roll_d = vel_roll - vel_des_roll;
        float error_pitch_d = vel_pitch - vel_des_pitch;
        float error_yaw_d = vel_yaw - vel_des_yaw; 

        // Calcular las entradas de control
        float u_roll = (-kproll * error_roll) - (kdroll * error_roll_d);
        float u_pitch = (-kppitch * error_pitch) - (kdpitch * error_pitch_d);
        float u_yaw = (-kpyaw * error_yaw) - (kdyaw * error_yaw_d);

        // Calcular los torques
        float torque_roll = Jxx * (((Jzz - Jyy) / Jxx) * error_pitch_d * error_yaw + u_roll);
        float torque_pitch = Jyy * (((Jxx - Jzz) / Jyy) * error_roll_d * error_yaw + u_pitch);
        float torque_yaw = Jzz * (((Jyy - Jxx) / Jzz) * error_roll_d * error_pitch + u_yaw);

        // Publicar los torques calculados
        geometry_msgs::Twist msg_roll;
        msg_roll.angular.z = torque_roll;
        pub_torque_roll.publish(msg_roll);

        geometry_msgs::Twist msg_pitch;
        msg_pitch.angular.z = torque_pitch;
        pub_torque_pitch.publish(msg_pitch);

        geometry_msgs::Twist msg_yaw;
        msg_yaw.angular.z = torque_yaw;
        msg_yaw.linear.z = error_yaw;
        pub_torque_yaw.publish(msg_yaw);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}