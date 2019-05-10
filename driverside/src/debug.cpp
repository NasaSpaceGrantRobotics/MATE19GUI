#include "../include/Debug/debug.h"
#include "ui_debug.h"

Debug::Debug(int argc, char **argv, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Debug),
    node(argc, argv)
{
    ui->setupUi(this);

    node.init();

    connect(&node, &QNode::refreshVelocity, this, &Debug::updateVelocity);
    connect(&node, &QNode::refreshAcceleration, this, &Debug::updateAcceleration);
    connect(&node, &QNode::rosShutdown, this, &Debug::close);

    qRegisterMetaType<QVector<float>>("QVector<float>");
}

Debug::~Debug()
{
    delete ui;
}

void Debug::updateVelocity(QVector<double> velocity)
{
    ui->labelVelocityXValue->setText(QString::number(velocity[0]));
    ui->labelVelocityYValue->setText(QString::number(velocity[1]));
    ui->labelVelocityZValue->setText(QString::number(velocity[2]));
}

void Debug::updateAcceleration(QVector<double> acceleration)
{
    ui->labelAccelerationXValue->setText(QString::number(acceleration[0]));
    ui->labelAccelerationYValue->setText(QString::number(acceleration[1]));
    ui->labelAccelerationZValue->setText(QString::number(acceleration[2]));
}

void Debug::updateSpecies(QVector<uint8_t> species)
{
    ui->labelStarValue->setText(QString::number(species[0]));
    ui->labelCircleValue->setText(QString::number(species[1]));
    ui->labelSquareValue->setText(QString::number(species[2]));
    ui->labelTriangleValue->setText(QString::number(species[3]));
}

void Debug::updateDirection(double direction)
{
    ui->dialDirection->setValue(direction);
}

void Debug::updateInduction(bool induction)
{
    if (induction)
    {
        ui->labelInductionValue->setText("<p style='color:green;'>True</p>");
    }
    else
    {
        ui->labelInductionValue->setText("<p style='color:red;'>False</p>");
    }
}

void Debug::updatePH(double ph)
{
    ui->labelPHValue->setText(QString::number(ph));
}

void Debug::updateTemp(double temp)
{
    ui->labelTempValue->setText(QString::number(temp) + " °C");
}

void Debug::updateGrid(QImage image)
{
    ui->labelGrid->setPixmap(QPixmap::fromImage(image));
}
