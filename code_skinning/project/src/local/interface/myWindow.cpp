#include "myWindow.hpp"

#include "myWidgetGL.hpp"
#include "../../lib/common/error_handling.hpp"
#include "ui_mainwindow.h"

#include <iostream>

myWindow::myWindow(QWidget *parent)
    :QMainWindow(parent),ui(new Ui::MainWindow)
{
    try
    {
        //Setup window layout
        ui->setupUi(this);

        //Create openGL context
        QGLFormat qglFormat;
        qglFormat.setVersion(1,2);

        //Create OpenGL Widget renderer
        glWidget=new myWidgetGL(qglFormat);

        //Add the OpenGL Widget into the layout
        ui->layout_scene->addWidget(glWidget);
    }
    catch(cpe::exception_cpe const& e)
    {
        std::cout<<std::endl<<e.report_exception()<<std::endl;
    }

    //Connect slot and signals
    connect(ui->quit,SIGNAL(clicked()),this,SLOT(action_quit()));
    connect(ui->draw,SIGNAL(clicked()),this,SLOT(action_draw()));
    connect(ui->wireframe,SIGNAL(clicked()),this,SLOT(action_wireframe()));
    connect(ui->skeleton,SIGNAL(clicked()),this,SLOT(action_skeleton()));
    connect(ui->mesh,SIGNAL(clicked()),this,SLOT(action_mesh()));

}

myWindow::~myWindow()
{}

void myWindow::action_quit()
{
    close();
}

void myWindow::action_draw()
{
    glWidget->change_draw_state();
}

void myWindow::action_wireframe()
{
    bool const state_wireframe=ui->wireframe->isChecked();
    glWidget->wireframe(state_wireframe);
}

void myWindow::action_skeleton()
{
    bool const state_skeleton=ui->skeleton->isChecked();
    glWidget->skeleton(state_skeleton);
}

void myWindow::action_mesh()
{
    bool const state_mesh=ui->mesh->isChecked();
    glWidget->mesh(state_mesh);
}

