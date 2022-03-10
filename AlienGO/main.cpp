#include "aliengo_main_window.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    AliengoMainWindow w;
    w.show();

    return a.exec();
}
