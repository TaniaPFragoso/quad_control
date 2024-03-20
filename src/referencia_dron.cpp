#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

int main(int argc, char** argv) {
    ros::init(argc, argv, "referencia_dron");
    ros::NodeHandle nh;

    // Solo dos publicadores para Twist
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/vel_des", 10);
    ros::Publisher pos_pub = nh.advertise<geometry_msgs::Twist>("/pos_des", 10);

    ros::Rate rate(100); // 100 Hz

    float t = 0;
    float contador = 0;
    float step = .01; // Incremento de tiempo por ciclo

    // Inicializa estas variables si es necesario
    float xdes = 0, ydes = 0, zdes = 0, psides = 0;
    float dxdes = 0, dydes = 0, dzdes = -0.5, dpsides = 0;

    while (ros::ok())
    {
        t = contador * step; // Actualiza el tiempo actual

        if (t >= 0 && t <= 5) {
            //dxdes = 0; dydes = 0; dzdes = -.5; dpsides = 0;
            xdes = xdes + step*dxdes;
            ydes = ydes + step*dydes;
            zdes = zdes + step*dzdes;
            psides = psides + step*dpsides;
        } 
        else if (t > 5 && t <= 65) {
            dxdes = 0.5 * sin(0.1 * (t - 5));
            dydes = 0.5 * cos(0.1 * (t - 5));
            dzdes = 0;
            dpsides = 0.1;
            xdes = xdes + step*dxdes;
            ydes = ydes + step*dydes;
            zdes = zdes + step*dzdes;
            psides = psides + step*dpsides;
        } 
        else if (t > 65) {
            break;
        }

        // Cálculos para actualización de posiciones y velocidades aquí

        // Prepara los mensajes
        geometry_msgs::Twist vel_msg;
        geometry_msgs::Twist pos_msg;

        // Asigna valores a los mensajes para velocidades
        vel_msg.linear.x = dxdes;
        vel_msg.linear.y = dydes;
        vel_msg.linear.z = dzdes;
        vel_msg.angular.z = dpsides;

        // Asigna valores a los mensajes para posiciones (Reutilizando Twist de manera no convencional)
        pos_msg.linear.x = xdes;
        pos_msg.linear.y = ydes;
        pos_msg.linear.z = zdes;
        pos_msg.angular.z = psides;

        // Publica los mensajes
        vel_pub.publish(vel_msg);
        pos_pub.publish(pos_msg);

        ROS_INFO("Tiempo: %f, X: %f, Y: %f, Z: %f, Psi: %f", t, xdes, ydes, zdes, psides);

        contador++;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}