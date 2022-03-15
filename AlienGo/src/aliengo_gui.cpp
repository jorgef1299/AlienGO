#include "aliengo_gui.h"
#include "ui_main_window.h"

namespace Aliengo {
    MainWindow::MainWindow(QWidget *parent) :
            QMainWindow(parent), ui(new Ui::MainWindow)
    {
        ui->setupUi(this);

        // Group Radio buttons in groups
        QButtonGroup* button_group_top_camera = new QButtonGroup();
        button_group_top_camera->addButton(ui->radioButton_Top_Camera_1);
        button_group_top_camera->addButton(ui->radioButton_Top_Camera_2);
        button_group_top_camera->addButton(ui->radioButton_Top_Camera_3);
        QButtonGroup* button_group_bottom_camera = new QButtonGroup();
        button_group_bottom_camera->addButton(ui->radioButton_Bottom_Camera_1);
        button_group_bottom_camera->addButton(ui->radioButton_Bottom_Camera_2);
        button_group_bottom_camera->addButton(ui->radioButton_Bottom_Camera_3);
        QButtonGroup* button_group_map = new QButtonGroup();
        button_group_map->addButton(ui->radioButton_Map_1);
        button_group_map->addButton(ui->radioButton_Map_2);
        button_group_map->addButton(ui->radioButton_Map_3);

        QVBoxLayout* new_vbox = new QVBoxLayout();
        FRender_panel = new rviz::RenderPanel();
        new_vbox->addWidget(FRender_panel);
        ui->GroupBox_Map->layout()->addWidget(FRender_panel);

        FManager = new rviz::VisualizationManager(FRender_panel);
        FRender_panel->initialize(FManager->getSceneManager(), FManager);
        FManager->initialize();
        FManager->startUpdate();
        FGrid = FManager->createDisplay("rviz/Grid", "adjustable grid", true);

        connect(button_group_top_camera, SIGNAL(buttonPressed(int)), this, SLOT(RadioButtonTopCameraPressed(int)));
        connect(button_group_bottom_camera, SIGNAL(buttonPressed(int)), this, SLOT(RadioButtonBottomCameraPressed(int)));
        connect(button_group_map, SIGNAL(buttonPressed(int)), this, SLOT(RadioButtonMapPressed(int)));

        // Initialize ROS Image Subscribers
        FTopCameraImageSub = new ros_qt_interface::TRosQtSensorMsgsImageSub("/top_camera/color/image_raw");
        connect(FTopCameraImageSub, SIGNAL(DataReceived()), this, SLOT(SLOT_ROS_NewTopCameraImage()));
        FBottomCameraImageSub = new ros_qt_interface::TRosQtSensorMsgsImageSub("/camera/color/image_raw");
        connect(FBottomCameraImageSub, SIGNAL(DataReceived()), this, SLOT(SLOT_ROS_NewBottomCameraImage()));
    }

    MainWindow::~MainWindow() {
        delete ui;
        delete FManager;
    }

    void MainWindow::RadioButtonTopCameraPressed(int button_id)
    {
        ROS_INFO("Top Camera: Button %d clicked...", button_id);
    }

    void MainWindow::RadioButtonBottomCameraPressed(int button_id)
    {
        ROS_INFO("Bottom Camera: Button %d clicked...", button_id);
    }

    void MainWindow::RadioButtonMapPressed(int button_id)
    {
        ROS_INFO("Map: Button %d clicked...", button_id);
    }

    void MainWindow::SLOT_ROS_NewTopCameraImage()
    {
        //ui->TopCam
        ROS_INFO("Recebi frame da câmara topo!");
        sensor_msgs::Image image;
        FTopCameraImageSub->GetData(image);
        QImage converted_image(&image.data[0], image.width, image.height, image.step, QImage::Format_RGB888);
        ui->TopCameraImage->setStyleSheet("");
        QPixmap pix_image = QPixmap::fromImage(converted_image);
        ui->TopCameraImage->setPixmap(pix_image);
    }

    void MainWindow::SLOT_ROS_NewBottomCameraImage()
    {
        ROS_INFO("Recebi frame da câmara de baixo!");
        sensor_msgs::Image image;
        FBottomCameraImageSub->GetData(image);
        QImage converted_image(&image.data[0], image.width, image.height, image.step, QImage::Format_RGB888);
        ui->BottomCameraImage->setStyleSheet("");
        QPixmap pix_image = QPixmap::fromImage(converted_image);
        ui->BottomCameraImage->setPixmap(pix_image);
    }
}