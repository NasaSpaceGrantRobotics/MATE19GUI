#ifndef QNODE_H
#define QNODE_H

#include <QImage>
#include <QThread>
#include <QVector>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

class QNode : public QThread
{
    Q_OBJECT

signals:
    void refreshVelocity(QVector<double>);
    void refreshAcceleration(QVector<double>);
    void refreshSpecies(QVector<uint8_t>);
    void refreshDirection(double);
    void refreshInduction(bool);
    void refreshPH(double);
    void refreshTemp(double);
    void refreshGrid(QImage);
    void rosShutdown();

public:
    QNode(int argc, char** argv);
    virtual ~QNode();
    bool init();
    void run();

private:
    int init_argc;
    char** init_argv;
    ros::Subscriber velocitySubscriber;
    ros::Subscriber accelerationSubscriber;
    ros::Subscriber speciesSubscriber;
    ros::Subscriber directionSubscriber;
    ros::Subscriber inductionSubscriber;
    ros::Subscriber phSubscriber;
    ros::Subscriber tempSubscriber;
    ros::Subscriber gridSubsciber;

    void updateVelocity(geometry_msgs::Vector3 velocity);
    void updateAcceleration(sensor_msgs::Imu acceleration);
    void updateSpecies(std_msgs::UInt8MultiArray species);
    void updateDirection(geometry_msgs::Quaternion direction);
    void updateInduction(std_msgs::Bool induction);
    void updatePH(std_msgs::Float64 ph);
    void updateTemp(std_msgs::Float64 temp);
    void updateGrid(sensor_msgs::Image image);

};

#endif // QNODE_H
