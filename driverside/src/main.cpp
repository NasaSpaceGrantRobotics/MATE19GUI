#include <QApplication>

#include "../include/Debug/debug.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    Debug w(argc, argv);
    w.show();

    return a.exec();
}
