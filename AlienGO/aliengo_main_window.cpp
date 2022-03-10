#include "aliengo_main_window.h"
#include "ui_aliengo_main_window.h"

AliengoMainWindow::AliengoMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::AliengoMainWindow)
{
    ui->setupUi(this);
}

AliengoMainWindow::~AliengoMainWindow()
{
    delete ui;
}
