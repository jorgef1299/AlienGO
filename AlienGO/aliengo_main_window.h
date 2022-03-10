#ifndef ALIENGO_MAIN_WINDOW_H
#define ALIENGO_MAIN_WINDOW_H

#include <QMainWindow>

namespace Ui {
class AliengoMainWindow;
}

class AliengoMainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit AliengoMainWindow(QWidget *parent = 0);
    ~AliengoMainWindow();

private:
    Ui::AliengoMainWindow *ui;
};

#endif // ALIENGO_MAIN_WINDOW_H
