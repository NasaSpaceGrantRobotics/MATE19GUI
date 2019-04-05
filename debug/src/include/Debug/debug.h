#ifndef DEBUG_H
#define DEBUG_H

#include <QPixmap>
#include <QVector>
#include <QWidget>
#include "qnode.h"

namespace Ui {
class Debug;
}

class Debug : public QWidget
{
    Q_OBJECT

public:
    explicit Debug(int argc, char **argv, QWidget *parent = nullptr);
    ~Debug();

private:
    Ui::Debug *ui;
    QNode node;

private slots:
    void updateVelocity(QVector<double> velocity);
    void updateAcceleration(QVector<double> acceleration);
    void updateSpecies(QVector<uint8_t> species);
    void updateDirection(double direction);
    void updateInduction(bool induction);
    void updatePH(double ph);
    void updateTemp(double temp);
    void updateGrid(QImage image);

};

#endif // DEBUG_H
