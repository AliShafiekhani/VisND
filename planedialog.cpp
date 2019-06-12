//----------------------------------------------//
//           N-D Visualization GUI              //
//                                              //
//        Copyright (c) ViGIR-Lab 2019          //
//          Written by Ali Shfiekhani           //
//----------------------------------------------//

#include "planedialog.h"
#include "ui_planedialog.h"

PlaneDialog::PlaneDialog(VisualizeND *visnd, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PlaneDialog),
    visnd_(visnd),
    actor_(vtkSmartPointer<vtkActor>::New()),
    planeSource(vtkSmartPointer<vtkPlaneSource>::New())
{
    setWindowFlags(Qt::FramelessWindowHint | Qt::WindowTitleHint);
    ui->setupUi(this);

    ui->XSlider->setMinimum(visnd_->min_pt_[0]);
    ui->XSlider->setMaximum(visnd_->max_pt_[0]);
    ui->XSlider->setValue(visnd_->min_pt_[0]);
    ui->YSlider->setMinimum(visnd_->min_pt_[1]);
    ui->YSlider->setMaximum(visnd_->max_pt_[1]);
    ui->YSlider->setValue(visnd_->min_pt_[1]);
    ui->ZSlider->setMinimum(visnd_->min_pt_[2]);
    ui->ZSlider->setMaximum(visnd_->max_pt_[2]);
    ui->ZSlider->setValue(visnd_->min_pt_[2]);

    ui->NxSlider->setMinimum(-100);
    ui->NxSlider->setMaximum(100);
    ui->NxSlider->setValue(0);
    ui->NySlider->setMinimum(-100);
    ui->NySlider->setMaximum(100);
    ui->NySlider->setValue(0);
    ui->NzSlider->setMinimum(-100);
    ui->NzSlider->setMaximum(100);
    ui->NzSlider->setValue(100);

//    ui->PlaneWidthLineEdit->setFixedWidth(100);
    ui->PlaneWidthLineEdit->setText("1000");
    ui->PlaneWidthLineEdit->setValidator((new QIntValidator(0, 100000, this)));

}

PlaneDialog::~PlaneDialog()
{
    delete ui;
}
void PlaneDialog::cleanPlane(){
    visnd_->ui_->qvtkWidget->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(actor_);
    visnd_->ui_->qvtkWidget->update();

}

void PlaneDialog::updatePlane(){
    // Calculate normal vector
    double nx = visnd_->Nx;
    double ny = visnd_->Ny;
    double nz = visnd_->Nz;
    double sumsqrt = sqrt(nx*nx+ny*ny+nz*nz);
    nx = nx / sumsqrt;
    ny = ny / sumsqrt;
    nz = nz / sumsqrt;

    double x0 = visnd_->X;
    double y0 = visnd_->Y;
    double z0 = visnd_->Z;

    vtkSmartPointer<vtkPlaneSource> planeSource0 = vtkSmartPointer<vtkPlaneSource>::New();
    planeSource0->SetCenter(x0,y0,z0);
    planeSource0->SetNormal(nx,ny,nz);
    planeSource0->Update();

    double p1[3],p2[3],vo[3];
    planeSource0->GetOrigin(vo);
    planeSource0->GetPoint1(p1);
    planeSource0->GetPoint2(p2);
    double v1[3], v2[3];
    unsigned int width = ui->PlaneWidthLineEdit->text().toInt();
    v1[0] = (p1[0]-vo[0]) * width + vo[0];
    v2[0] = (p2[0]-vo[0]) * width + vo[0];
    v1[1] = (p1[1]-vo[1]) * width + vo[1];
    v2[1] = (p2[1]-vo[1]) * width + vo[1];
    v1[2] = (p1[2]-vo[2]) * width + vo[2];
    v2[2] = (p2[2]-vo[2]) * width + vo[2];

//    vtkSmartPointer<vtkPlaneSource> planeSource = vtkSmartPointer<vtkPlaneSource>::New();
    planeSource->SetOrigin(x0,y0,z0);
    planeSource->SetPoint1(v1);
    planeSource->SetPoint2(v2);
    planeSource->SetCenter(vo);
    planeSource->SetXResolution(10);
    planeSource->SetYResolution(10);

    planeSource->Update();
    vtkPolyData* plane = planeSource->GetOutput();
    // Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    #if VTK_MAJOR_VERSION <= 5
      mapper->SetInput(plane);
    #else
      mapper->SetInputData(plane);
    #endif
    actor_->SetMapper(mapper);
    actor_->GetProperty()->SetColor(1,0,0);
    actor_->GetProperty()->SetEdgeColor(1,0,0);
    actor_->GetProperty()->SetAmbient(0.9);
    actor_->GetProperty ()->SetDiffuse (0.9);
    actor_->GetProperty ()->SetSpecular (0.1);
    actor_->Modified();

    visnd_->ui_->qvtkWidget->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(actor_);
    visnd_->ui_->qvtkWidget->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(actor_);

    visnd_->ui_->qvtkWidget->update();


}

void PlaneDialog::on_XSlider_sliderMoved(int position)
{
    visnd_->X = position;
    updatePlane();
}

void PlaneDialog::on_YSlider_sliderMoved(int position)
{
    visnd_->Y = position;
    updatePlane();
}

void PlaneDialog::on_ZSlider_sliderMoved(int position)
{
    visnd_->Z = position;
    updatePlane();
}

void PlaneDialog::on_NzSlider_sliderMoved(int position)
{
    visnd_->Nz = position / 100.0;
    updatePlane();

}

void PlaneDialog::on_NySlider_sliderMoved(int position)
{
    visnd_->Ny = position / 100.0;
    updatePlane();
}

void PlaneDialog::on_NxSlider_sliderMoved(int position)
{
    visnd_->Nx = position / 100.0;
    updatePlane();
}

void PlaneDialog::on_buttonBox_accepted()
{
    double N[3];
    planeSource->GetNormal(N);
    visnd_->Nx = N[0];
    visnd_->Ny = N[1];
    visnd_->Nz = N[2];
    double XYZ0[3];
    planeSource->GetCenter(XYZ0);
    visnd_->X = XYZ0[0];
    visnd_->Y = XYZ0[1];
    visnd_->Z = XYZ0[2];
    // TODO: create ground point cloud
    for(list< InstanceCloud<PointType> >::iterator it = visnd_->frames_.begin(); it != visnd_->frames_.end(); ++it){
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
        for(unsigned int i = 0; i < it->getCloudPtr()->points.size(); i++){
            PointType xyz = it->getCloudPtr()->points[i];
           double x = xyz.x;
           double y = xyz.y;
           double z = xyz.z;
           double h = N[0] * (x - XYZ0[0]) + N[1] * (y - XYZ0[1]) + N[2] * (z - XYZ0[2]);
           if(h <= 0){
               cloud->points.push_back(xyz);
           }

        }
        it->getCloudPtr()->points.resize(cloud->points.size());
        pcl::copyPointCloud(*cloud, *(it->getCloudPtr()));


    }
    cleanPlane();
}

void PlaneDialog::on_buttonBox_rejected()
{
    cleanPlane();
}


void PlaneDialog::on_pushButton_clicked()
{
    visnd_->on_actionSelect_Points_triggered(true);
    visnd_->ui_->actionSelect_Points->setChecked(true);
    while(visnd_->framesSelectedPoints_[visnd_->frameIndx_]->points.size() < 3){
        visnd_->sleepFor(500);
    }
    double x0,x1,x2, y0,y1,y2, z0,z1,z2;
    x0 = visnd_->framesSelectedPoints_[visnd_->frameIndx_]->points[0].x;
    x1 = visnd_->framesSelectedPoints_[visnd_->frameIndx_]->points[1].x;
    x2 = visnd_->framesSelectedPoints_[visnd_->frameIndx_]->points[2].x;
    visnd_->X = (x0 + x1 + x2) / 3.0;
    ui->XSlider->setValue(visnd_->X);

    y0 = visnd_->framesSelectedPoints_[visnd_->frameIndx_]->points[0].y;
    y1 = visnd_->framesSelectedPoints_[visnd_->frameIndx_]->points[1].y;
    y2 = visnd_->framesSelectedPoints_[visnd_->frameIndx_]->points[2].y;
    visnd_->Y = (y0+y1+y2)/3.0;
    ui->YSlider->setValue(visnd_->Y);

    z0 = visnd_->framesSelectedPoints_[visnd_->frameIndx_]->points[0].z;
    z1 = visnd_->framesSelectedPoints_[visnd_->frameIndx_]->points[1].z;
    z2 = visnd_->framesSelectedPoints_[visnd_->frameIndx_]->points[2].z;
    visnd_->Z = (z0+z1+z2) / 3.0;
    ui->ZSlider->setValue(visnd_->Z);

    double nx, ny, nz;
    nx = (y1 - y0) * (z2 - z0) - (z1 - z0) * (y2 - y0);
    ny = (x2 - x0) * (z1 - z0) - (x1 - x0) * (z2 - z0);
    nz = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0);
    double sumsqrt = sqrt(nx*nx+ny*ny+nz*nz);
    nx = nx / sumsqrt;
    ny = ny / sumsqrt;
    nz = nz / sumsqrt;

    visnd_->Nx = nx;
    visnd_->Ny = ny;
    visnd_->Nz = nz;
    ui->NxSlider->setValue(100 * nx);
    ui->NySlider->setValue(100 * ny);
    ui->NzSlider->setValue(100 * nz);

    vtkSmartPointer<vtkPlaneSource> planeSource = vtkSmartPointer<vtkPlaneSource>::New();
    planeSource->SetOrigin(x0,y0,z0);
    planeSource->SetPoint1(x1,y1,z1);
    planeSource->SetPoint2(x2,y2,z2);
    planeSource->SetXResolution(10);
    planeSource->SetYResolution(10);
    planeSource->SetNormal(nx,ny,nz);
    planeSource->Update();
    vtkPolyData* plane = planeSource->GetOutput();
    // Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    #if VTK_MAJOR_VERSION <= 5
      mapper->SetInput(plane);
    #else
      mapper->SetInputData(plane);
    #endif
    actor_->SetMapper(mapper);
    actor_->GetProperty()->SetColor(1,0,0);
    actor_->GetProperty()->SetEdgeColor(1,0,0);
    actor_->GetProperty()->SetAmbient(0.9);
    actor_->GetProperty ()->SetDiffuse (0.9);
    actor_->GetProperty ()->SetSpecular (0.9);
    actor_->Modified();

    visnd_->ui_->qvtkWidget->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(actor_);
    visnd_->ui_->qvtkWidget->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(actor_);

    visnd_->ui_->qvtkWidget->update();

    visnd_->ui_->actionSelect_Points->setChecked(false);
    visnd_->on_actionSelect_Points_triggered(false);

}
