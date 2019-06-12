//----------------------------------------------//
//           N-D Visualization GUI              //
//                                              //
//        Copyright (c) ViGIR-Lab 2019          //
//          Written by Ali Shfiekhani           //
//----------------------------------------------//

#ifndef PLANEDIALOG_H
#define PLANEDIALOG_H

#include <math.h>

#include <QDialog>

#include <vtkVersion.h>
#include <vtkPlaneSource.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>


#include "visualize_nd.h"

#define PI 3.14159265

class VisualizeND;

namespace Ui {
class PlaneDialog;
}

class PlaneDialog : public QDialog
{

    Q_OBJECT

public:
    explicit PlaneDialog(VisualizeND *visnd, QWidget *parent = 0);
    ~PlaneDialog();

public:
    void updatePlane();
    void cleanPlane();


private slots:
    void on_XSlider_sliderMoved(int position);

    void on_YSlider_sliderMoved(int position);

    void on_ZSlider_sliderMoved(int position);

    void on_buttonBox_accepted();

    void on_buttonBox_rejected();

    void on_pushButton_clicked();

    void on_NzSlider_sliderMoved(int position);

    void on_NySlider_sliderMoved(int position);

    void on_NxSlider_sliderMoved(int position);

private:
    Ui::PlaneDialog *ui;
    VisualizeND *visnd_;
    vtkSmartPointer<vtkActor> actor_;
    vtkSmartPointer<vtkPlaneSource> planeSource;
};

#endif // PLANEDIALOG_H


