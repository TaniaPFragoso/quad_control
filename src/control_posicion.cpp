#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>

float masa=2;
float Th;

//Variables provenientes de dinamica

    //Posiciones actuales lineales.
    float x;
    float y;
    float z;

    //Velocidades actuales lineales.
    float vel_x;
    float vel_y;
    float vel_z;

    //Posicion angular del dron.
    float roll;
    float pitch;
    float yaw;

//Variables provenientes de refenrencia

    //Posiciones deseadas.
    float x_des;
    float y_des;
    float z_des;

    //Velocidades deseadas (velocidad lineal).
    float vel_des_x;
    float vel_des_y;
    float vel_des_z;
    float vel_des_yaw;


    //POsicion angular del dron deseada.
    float roll_des;
    float pitch_des;
    float yaw_des;



//Controles auxiliares
float uvx;
float uvy;
float uvz;

//Errores lineales
float ex;
float ey;
float ez;

//Errores de velocidad lineales
float ex_p;
float eyp;
float ezp;

//Ganancias de control

    //Ganancias de control lineal kp de X Y y Z
    float kpx=0.4;
    float kpy=0.4;
    float kpz=0.4;

    //Ganancias de control lineal kd X Y y Z
    float kdx=1;
    float kdy=1;
    float kdz=1;


void pos_desCallback(const geometry_msgs::Twist::ConstPtr& posd) //Recibidor de posiciones deseadas. Referencia
{
    x_des = posd->linear.x;
    y_des = posd->linear.y;
    z_des = posd->linear.z;
    yaw_des = posd->angular.z;

    
}

void vel_desCallback(const geometry_msgs::Twist::ConstPtr& veld) //Recibidor de velocidades deseadas. Referencia
{
    vel_des_x = veld-> linear.x;
    vel_des_y = veld -> linear.y;
    vel_des_z = veld -> linear.z;
    vel_des_yaw = veld -> angular.z;
}

void posCallback(const geometry_msgs::Twist::ConstPtr& pos) //Recibidor de posiciones lineales. Dinamica
{
    x = pos-> linear.x;
    y = pos -> linear.y;
    z = pos -> linear.z;
    roll=pos-> angular.x;
    pitch=pos -> angular.y;
    yaw= pos-> angular.z;
    
}

void velCallback(const geometry_msgs::Twist::ConstPtr& vel) //Recibidor de velocidad lineales. Dinamica
{
    vel_x = vel-> linear.x;
    vel_y = vel -> linear.y;
    vel_z = vel -> linear.z;
    
}

int main(int argc, char **argv)
{
    // Inicializar ROS y la creación del nodo
    ros::init(argc, argv, "control_posicion");
    ros::NodeHandle nh; 
    
    // Declaración de los suscribers
    ros::Subscriber posicion_deseada_sub = nh.subscribe("/pos_des", 10, &pos_desCallback); //Topico de posiciones deseadas (viene de referencias)
    ros::Subscriber velocidades_deseadas_sub = nh.subscribe("/vel_des", 10, &vel_desCallback);//Topico de velocidades deseadas (viene de referencias)
    ros::Subscriber posicion_sub = nh.subscribe("/posicion_linear_angular", 10, &posCallback);//Topico de velocidades (viene de dinamica)
    ros::Subscriber velocidades_sub = nh.subscribe("/velocidad_linear_angular", 10, &velCallback);//Topico de velocidades (viene de dinamica)

    //DEclaracion de publishers
    ros::Publisher errores_lineales_pub = nh.advertise<geometry_msgs::Twist>("/errores_lineales", 10);
    ros::Publisher variables_pub = nh.advertise<geometry_msgs::Twist>("/thrust_rolldes_pitchdes", 10);

    geometry_msgs::Twist errores_var;
    geometry_msgs::Twist th_angulos_var;

    
    // Frecuencia a la que correrá el nodo en Hz
    ros::Rate loop_rate(100);    // ros::Rate <$NOMBRE DE LA VARIABLE>($<Hz>)


    while(ros::ok())
    {

        //Error de posiciones, X, Y y Z
        ex=x-x_des;
        ey=y-y_des;
        ez=z-z_des;

        //Error de velocidades X, Y y Z
        ex_p=vel_x - vel_des_x;
        eyp=vel_y - vel_des_y;
        ezp=vel_z - vel_des_z;


        //Controles virtuales
        uvx = -(kpx*ex)-(kdx*ex_p);
        uvy = -(kpy*ey)-(kdy*eyp);
        uvz = -(kpz*ez)-(kdz*ezp);

        //Th

        Th = (masa/(cos(roll)*cos(pitch)))*(-9.81 + uvz);

        //Roll deseado

        roll_des = asin ((masa/Th)*(sin(yaw_des)*uvx - cos(yaw_des)*uvy));

        //Pitch deseado

        pitch_des = asin (((masa/Th)*uvx-sin(yaw_des)*sin(roll_des))/(cos(yaw_des)*cos(roll_des)));

        // Mandar la publicación de los mensajes

        //Errores lineales
        errores_var.linear.x = ex; //Error en x
        errores_var.linear.y = ey; //Error en y
        errores_var.linear.z = ez; //Error en z

        errores_var.angular.x = ex_p; //Derivada del error x
        errores_var.angular.y = eyp; //DErivada del error y
        errores_var.angular.z = ezp; //DErivada del error z

        //Thrust roll y pitch deseados

        th_angulos_var.linear.x = Th; //Thrust
        th_angulos_var.linear.y = roll_des; //Roll desdeado
        th_angulos_var.linear.z = pitch_des; //Pitch deseado


        errores_lineales_pub.publish(errores_var);
        variables_pub.publish(th_angulos_var);


        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}