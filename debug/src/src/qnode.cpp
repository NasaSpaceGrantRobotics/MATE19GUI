#include "../include/Debug/qnode.h"

QNode::QNode(int argc, char** argv ) :
        init_argc(argc),
        init_argv(argv)
{
}

QNode::~QNode()
{
    if (ros::isStarted())
    {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init()
{
    ros::init(init_argc,init_argv, "debug");
    if (!ros::master::check())
    {
        return false;
    }

    ros::start(); // explicitly needed since our nodehandle is going out of scope.

    ros::NodeHandle n;
    velocitySubscriber = n.subscribe("/velocity", 1000, &QNode::updateVelocity, this);
    accelerationSubscriber = n.subscribe("/acceleration", 1000, &QNode::updateAcceleration, this);
    speciesSubscriber = n.subscribe("/species", 1000, &QNode::updateSpecies, this);
    directionSubscriber = n.subscribe("/direction", 1000, &QNode::updateDirection, this);
    inductionSubscriber = n.subscribe("/induction", 1000, &QNode::updateInduction, this);
    phSubscriber = n.subscribe("/ph", 1000, &QNode::updatePH, this);
    tempSubscriber = n.subscribe("/temp", 1000, &QNode::updateTemp, this);
    gridSubsciber = n.subscribe("/grid", 1000, &QNode::updateGrid, this);

    start();
    return true;
}

void QNode::run()
{
    ros::NodeHandle n;
    velocitySubscriber = n.subscribe("/velocity", 1000, &QNode::updateVelocity, this);
    accelerationSubscriber = n.subscribe("/acceleration", 1000, &QNode::updateAcceleration, this);
    speciesSubscriber = n.subscribe("/species", 1000, &QNode::updateSpecies, this);
    directionSubscriber = n.subscribe("/direction", 1000, &QNode::updateDirection, this);
    inductionSubscriber = n.subscribe("/induction", 1000, &QNode::updateInduction, this);
    phSubscriber = n.subscribe("/ph", 1000, &QNode::updatePH, this);
    tempSubscriber = n.subscribe("/temp", 1000, &QNode::updateTemp, this);
    gridSubsciber = n.subscribe("/grid", 1000, &QNode::updateGrid, this);

    ros::spin();

    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::updateVelocity(geometry_msgs::Vector3 velocity)
{
    QVector<double> vector = { velocity.x, velocity.y, velocity.z };
    emit refreshVelocity(vector);
}

void QNode::updateAcceleration(sensor_msgs::Imu acceleration)
{
    geometry_msgs::Vector3 acceleration_vector = acceleration.linear_acceleration;
    QVector<double> vector = { acceleration_vector.x, acceleration_vector.y, acceleration_vector.z };
    emit refreshAcceleration(vector);
}

void QNode::updateSpecies(std_msgs::UInt8MultiArray species)
{
    QVector<uint8_t> spec = { species.data[0], species.data[1], species.data[2], species.data[3] };
    emit refreshSpecies(spec);
}

void QNode::updateDirection(geometry_msgs::Quaternion direction)
{
    // TODO
    double dir = 0;
    emit refreshDirection(dir);
}

void QNode::updateInduction(std_msgs::Bool induction)
{
    emit refreshInduction(induction.data);
}

void QNode::updatePH(std_msgs::Float64 ph)
{
    emit refreshPH(ph.data);
}

void QNode::updateTemp(std_msgs::Float64 temp)
{
    emit refreshTemp(temp.data);
}

void QNode::updateGrid(sensor_msgs::Image image)
{
    QImage img(image.data.data(), image.width, image.height, QImage::Format_RGB888);
    emit refreshGrid(img);
}
