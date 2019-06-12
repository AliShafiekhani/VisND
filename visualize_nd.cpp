//----------------------------------------------//
//           N-D Visualization GUI              //
//                                              //
//        Copyright (c) ViGIR-Lab 2019          //
//          Written by Ali Shfiekhani           //
//----------------------------------------------//

# include "visualize_nd.h"

VisualizeND::VisualizeND(QWidget *parent):
    QMainWindow(parent),
    ui_(new Ui::VisualizeND),
    clickedPoints_(new pcl::PointCloud<PointType>),
    gridSize_(0.1f),
    play_(false),
    rate_(1.0),
    tempAvailable_(false),
    tfAvailable_(false),
    colormap_(cv::COLORMAP_HOT),
    colorBar_(256, 20, CV_8UC1)
    {
        ui_->setupUi(this);
        this->setWindowTitle("VisND");
        this->setWindowIcon(QIcon("../resource/icons/VisND-icon.png"));

        pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

        // Initialize Widgets
        ui_->progressBar->setMinimum(0);
        ui_->progressBar->setMaximum(0);
        ui_->progressBar->hide();
        ui_->statusBar->addPermanentWidget(ui_->progressBar);

        // Action Group for speed rate
        QActionGroup *rateGroup = new QActionGroup(this);
        rateGroup->setExclusive(true);
        rateGroup->addAction(ui_->action_Rate_0_25);
        rateGroup->addAction(ui_->action_Rate_0_5);
        rateGroup->addAction(ui_->action_Rate_1_0);
        rateGroup->addAction(ui_->action_Rate_2_0);
        rateGroup->addAction(ui_->action_Rate_4_0);
        ui_->action_Rate_1_0->setChecked(true);

        // Action Group for Color Map
        QActionGroup *colorMapGroup = new QActionGroup(this);
        colorMapGroup->setExclusive(true);
        colorMapGroup->addAction(ui_->actionAutumn);
        colorMapGroup->addAction(ui_->actionBone);
        colorMapGroup->addAction(ui_->actionJet);
        colorMapGroup->addAction(ui_->actionWinter);
        colorMapGroup->addAction(ui_->actionRainbow);
        colorMapGroup->addAction(ui_->actionOcean);
        colorMapGroup->addAction(ui_->actionSummer);
        colorMapGroup->addAction(ui_->actionSpring);
        colorMapGroup->addAction(ui_->actionCool);
        colorMapGroup->addAction(ui_->actionHSV);
        colorMapGroup->addAction(ui_->actionPink);
        colorMapGroup->addAction(ui_->actionHot);
        ui_->actionHot->setChecked(true);


        // Color Bar
        for(int i = 0; i < 256; i++){
            colorBar_.row(i).colRange(0,20) = 255 - i;
        }
        updateColorBar();
        ui_->labelColorMap->hide();
        ui_->labelMaxTemp->hide();
        ui_->labelMinTemp->hide();
        ui_->lineEditMaxTemp->hide();
        ui_->lineEditMaxTemp->setFixedWidth(30);
        ui_->lineEditMaxTemp->setText("50");
        ui_->lineEditMaxTemp->setValidator(new QIntValidator(-100, 100, this));
        ui_->lineEditMinTemp->hide();
        ui_->lineEditMinTemp->setFixedWidth(30);
        ui_->lineEditMinTemp->setText("20");
        ui_->lineEditMinTemp->setValidator(new QIntValidator(-100, 100, this));
        ui_->horizontalLayoutMaxTemp->setAlignment(Qt::AlignBottom);
        ui_->horizontalLayoutMinTemp->setAlignment(Qt::AlignTop);

        ui_->actionColor_Bar->setChecked(false);

        // RGB/Temperature Rendering
        QActionGroup *renderingGroup = new QActionGroup(this);
        renderingGroup->setExclusive(true);
        renderingGroup->addAction(ui_->actionRGB);
        renderingGroup->addAction(ui_->actionTemperature);
        ui_->actionRGB->setChecked(true);

        // Set up the QVTK Window
        viewer_.reset(new pcl::visualization::PCLVisualizer ("5D Visualizer", false));
        ui_->qvtkWidget->SetRenderWindow(viewer_->getRenderWindow());
        viewer_->setupInteractor(ui_->qvtkWidget->GetInteractor(), ui_->qvtkWidget->GetRenderWindow());
        ui_->qvtkWidget->update();

    }

VisualizeND::~VisualizeND(){
    delete ui_;
}

void VisualizeND::pointPickingEventOccurred(const pcl::visualization::PointPickingEvent &event, void* visClassVoidPtr){
    VisualizeND * classObjPtr = static_cast< VisualizeND *>(visClassVoidPtr);
    if(event.getPointIndex() ==  -1){
        return;
    }

    if(classObjPtr->ui_->actionSelect_Points->isChecked()){
        unsigned int frameID = classObjPtr->frameIndx_;
        PointType pickedPoint;
        event.getPoint(pickedPoint.x,pickedPoint.y,pickedPoint.z);

        classObjPtr->framesSelectedPoints_[frameID]->points.push_back(pickedPoint);
        pcl::visualization::PointCloudColorHandlerCustom<PointType> red(classObjPtr->framesSelectedPoints_[frameID], 255,0,0);
        classObjPtr->viewer_->removePointCloud("clicked_points");
        classObjPtr->viewer_->addPointCloud(classObjPtr->framesSelectedPoints_[frameID], red, "clicked_points");
        classObjPtr->viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,10, "clicked_points");
    }
}

void VisualizeND::keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* visClassVoidPtr){
    VisualizeND * classObjPtr = static_cast< VisualizeND *>(visClassVoidPtr);
//    pcl::visualization::PCLVisualizer::Ptr viewer = classObjPtr->getViewer();
    static FrameIterator frameIt = classObjPtr->getFrameIterator();
    unsigned int N = classObjPtr->getNumFrames();
    static bool visRGB = !classObjPtr->ui_->actionTemperature->isChecked();
    if(event.getKeySym() == "t" && event.keyDown()){
        if(visRGB){
            classObjPtr->ui_->actionTemperature->setChecked(true);
            classObjPtr->updateView();
        }
        else{
            classObjPtr->ui_->actionRGB->setChecked(true);
            classObjPtr->updateView();
        }
        visRGB = !visRGB;
    }
    static unsigned int i = 0;
    if((event.getKeySym() == "Left" || event.getKeySym() == "Down") && event.keyDown()){

        if(i>0){
            i--;
            classObjPtr->on_actionPrevious_triggered();
        }
    }
    if( (event.getKeySym() == "Right" || event.getKeySym() == "Up") && event.keyDown()){
        if(i < N-1){
            i++;
            classObjPtr->on_actionNext_triggered();
        }
    }

    // TODO: add key to pause and play/resume
    // TODO: add measureing tool
    // TODO: add key to select points and show some metrics e.g. height, temperature, LAI
    // TODO: segment plants based on families given x,y corners of the family's rectangle
}

void VisualizeND::addInstaceCloud(InstanceCloud<PointType> instance_cloud){
    frames_.push_back(instance_cloud);
    frameIt_ = frames_.begin();
    nFrames_ = frames_.size();
}

bool VisualizeND::loadFrames(string files_path){

    QDir cloud_path(QString((files_path + "/Clouds").c_str()));
    QDir temp_path(QString((files_path + "/Temp").c_str()));
    QDir tf_path(QString((files_path + "/TF").c_str()));

    if(!cloud_path.exists() && !temp_path.exists() && !tf_path.exists()){

        QMessageBox::critical(this, "Error", "Failed to open Clouds, Temp, and TF directories! Make sure these sub-directories exist");
        return false;
    }
    if(cloud_path.exists() && !temp_path.exists()){
        QMessageBox::warning(this, "Warning", "No Temp! Skipping temp information!");
    }

    if(cloud_path.exists() && !tf_path.exists()){
        QMessageBox::warning(this, "Warning", "No TF directories! Skipping tf information!");
    }

    cloud_path.setFilter(QDir::Files);
    temp_path.setFilter(QDir::Files);
    tf_path.setFilter(QDir::Files);

    unsigned int n = cloud_path.count();
    if(n == 0){
        QMessageBox::critical(this, "Error", "No cloud file exists!");
        return false;
    }

    QStringList temps;
    QStringList tfs;
    if(temp_path.exists()){
        if(n != temp_path.count()){
            QMessageBox::warning(this, "Error", "Temperature files not matched with Cloud files");
        }
        else{
            tempAvailable_ = true;
            temps = temp_path.entryList(QStringList() << "*.txt", QDir::Files);
        }
    }
    if(tf_path.exists()){
        if(n != tf_path.count()){
            QMessageBox::warning(this, "Error", "TF files not matched with Cloud files");
        }
        else{
            tfAvailable_ = true;
            tfs = tf_path.entryList(QStringList() << "*.txt", QDir::Files);
        }
    }

    QStringList clouds = cloud_path.entryList(QStringList() << "*.ply" << "*.pcd", QDir::Files);

    ui_->statusBar->show();
    ui_->statusBar->showMessage("Loading Frames ...");
    ui_->progressBar->setMinimum(0);
    ui_->progressBar->setMaximum(n-1);
    ui_->progressBar->show();
    ui_->frameSlider->setMaximum(n-1);


    for(int i = 0; i < n; i++){
        double min_temp =  1000;
        double max_temp = -1000;
        QString cloud_name = cloud_path.absoluteFilePath(clouds.at(i));
        // load point cloud from disk
        typename pcl::PointCloud<PointType>::Ptr cloud_ptr(new pcl::PointCloud<PointType>);
        typename pcl::PointCloud<PointType>::Ptr cloud_filtered_ptr(new pcl::PointCloud<PointType>);
        if(QFileInfo(cloud_name).suffix() ==  "ply"){
            pcl::PLYReader ply_reader;
            if(ply_reader.read(cloud_name.toStdString(), *cloud_ptr) == -1){
                cout << "Error: Couldn't read " << cloud_name.toStdString() << endl;
                QMessageBox::critical(this, "Error", "Couldn't read ply file");
                return false;
            }
        }
        else if(QFileInfo(cloud_name).suffix() ==  "pcd"){
            pcl::PCDReader pcd_reader;
            if(pcd_reader.read(cloud_name.toStdString(), *cloud_ptr) == -1){
                cout << "Error: Couldn't read " << cloud_name.toStdString() << endl;
                QMessageBox::critical(this, "Error", "Couldn't read pcd file");
                return false;
            }
        }
        grid_.setInputCloud(cloud_ptr);
        grid_.setLeafSize (gridSize_, gridSize_, gridSize_);
        grid_.filter(*cloud_filtered_ptr);
        if(i == 0){
            pcl::getMinMax3D(*cloud_filtered_ptr, min_pt_, max_pt_);
        }
        cv::Mat cloud_temp(cloud_filtered_ptr->points.size(), 1, cv::DataType<double>::type);

        // load temperatures
        if(tempAvailable_){
            QString temp_name = temp_path.absoluteFilePath(temps.at(i));
            if(QFileInfo(temp_name).suffix() == "txt"){
                QFile temp_file(temp_name);
                if(!temp_file.open(QIODevice::ReadOnly)){
                    QMessageBox::critical(this, "Error", "Couldn't open Temperature file");
                    return false;
                }
                QTextStream in(&temp_file);
                unsigned int j = 0;
                while(!in.atEnd()){
                    double t = in.readLine().toDouble();
                    if(t > max_temp)
                        max_temp = t;
                    if(t < min_temp)
                        min_temp = t;
                    cloud_temp.at<double>(j,0) = t;
                    j++;
                }
                temp_file.close();
                min_max_temp_.push_back(make_pair(min_temp, max_temp));

            }
        }

        // read transformations
        Eigen::Matrix4f tfi = Eigen::Matrix4f::Identity();
        if(tfAvailable_){
            QString tf_name = tf_path.absoluteFilePath(tfs.at(i));
            if(QFileInfo(tf_name).suffix() == "txt"){
                QFile tf_file(tf_name);
                if(!tf_file.open(QIODevice::ReadOnly)){
                    QMessageBox::critical(this, "Error", "Couldn't open TF file");
                    return false;
                }
                QTextStream in(&tf_file);
                //     | h1  h2  h3  h4  |
                // H = | h5  h6  h7  h8  |
                //     | h9  h10 h11 h12 |
                //     | h13 h14 h15 h16 |
                int j = 0;
                for(; j < 16 && !in.atEnd(); j++){
                    float hi = in.readLine().toFloat();
                    int r = j/4;
                    int c = j%4;
                    tfi(r,c) = hi;
                }
                if (j < 16){
                    QMessageBox::critical(this, "Error", "TF file should have 16 rows of elements!");
                    return false;
                }
                tf_file.close();
            }
        }

        InstanceCloud<PointType> instance_cloud(cloud_filtered_ptr, cloud_temp, tfi, i);
        frames_.push_back(instance_cloud);
        ui_->progressBar->setValue(i);

    }
    ui_->progressBar->hide();
    ui_->statusBar->hide();

    frameIt_ = frames_.begin();
    frameIndx_ = 0;
    nFrames_ = frames_.size();
    return true;
}

pcl::visualization::PCLVisualizer::Ptr VisualizeND::getViewer(){return viewer_;}

FrameIterator VisualizeND::getFrameIterator(){return frameIt_;}

unsigned int VisualizeND::getNumFrames(){return nFrames_;}

void VisualizeND::init(){
    viewer_->setBackgroundColor (255, 255, 255);
    viewer_->setSize(800, 600);
    viewer_->addPointCloud<PointType>(frameIt_->getCloudPtr(), "cloud");
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");  // TODO: make the size of points modular
    viewer_->addText("Frame: 0" , 20, 20, 10, 1, 0, 0,"frame_id");
    viewer_->addText("RGB Rendering", 20, 30, 10, 1, 0, 0, "rgb/temp");
//    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 20, "frame_id");
    viewer_->resetCamera();
    viewer_->registerKeyboardCallback(keyboardEventOccurred, this);
    viewer_->registerPointPickingCallback(pointPickingEventOccurred, this);
    ui_->qvtkWidget->update();

    ui_->frameSlider->setValue(frameIndx_);

}

void VisualizeND::updateView(){

    if(tempAvailable_ && ui_->actionTemperature->isChecked()){
        cv::Mat temp_bgr;

        cv::Mat cloud_temp = frameIt_->getCloudTempCV();
        double min_temp = min_max_temp_[frameIndx_].first;
        double max_temp = min_max_temp_[frameIndx_].second;
        cloud_temp = (cloud_temp - min_temp)/(max_temp - min_temp)*255.0;
        cloud_temp.convertTo(cloud_temp, CV_8UC1);
        cv::applyColorMap(cloud_temp, temp_bgr, colormap_);

        pcl::PointCloud<PointType>::Ptr temp_cloud(new pcl::PointCloud<PointType>);
        pcl::copyPointCloud(*(frameIt_->getCloudPtr()), *temp_cloud);

        for(int i = 0; i < temp_cloud->points.size(); i++){
            temp_cloud->points[i].r = temp_bgr.at<cv::Vec3b>(i,0)[2];
            temp_cloud->points[i].g = temp_bgr.at<cv::Vec3b>(i,0)[1];
            temp_cloud->points[i].b = temp_bgr.at<cv::Vec3b>(i,0)[0];
        }
        pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb_temp_vis(temp_cloud);
        viewer_->updatePointCloud(temp_cloud, rgb_temp_vis, "cloud");
        viewer_->updateText("Temperature Rendering", 20, 30, 10, 1, 0, 0, "rgb/temp");

    }
    else{
        viewer_->updatePointCloud(frameIt_->getCloudPtr(), "cloud");
        viewer_->updateText("RGB Rendering", 20, 30, 10, 1, 0, 0, "rgb/temp");

    }
    if(ui_->actionSelect_Points->isChecked()){
        pcl::visualization::PointCloudColorHandlerCustom<PointType> red(framesSelectedPoints_[frameIndx_], 255,0,0);
        viewer_->removePointCloud("clicked_points");
        viewer_->addPointCloud(framesSelectedPoints_[frameIndx_], red, "clicked_points");
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,10, "clicked_points");
    }
    else{
        viewer_->removePointCloud("clicked_points");
    }
    viewer_->updateText("Frame: " + to_string(frameIndx_), 20, 20, 10, 1, 0, 0, "frame_id");
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

    ui_->qvtkWidget->update();
    ui_->frameSlider->setValue(frameIndx_);
}

void VisualizeND::updateColorBar(){
    cv::Mat img;
    cv::applyColorMap(colorBar_, img, colormap_);
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
    ui_->labelColorMap->setPixmap(QPixmap::fromImage(QImage(img.data, img.cols, img.rows, img.step, QImage::Format_RGB888)));
}

void VisualizeND::sleepFor(size_t milisec){
    size_t t2 = QDateTime::currentMSecsSinceEpoch() + milisec;
    while(t2 > QDateTime::currentMSecsSinceEpoch()){
        QApplication::processEvents(QEventLoop::AllEvents, 100);
    }
}

void VisualizeND::on_actionOpen_triggered()
{
     QString directory_name = QFileDialog::getExistingDirectory(this,
                              "Select Directory of Frames");
     filesPath_ = directory_name.toStdString();
     loadFrames(filesPath_);
     init();

}

void VisualizeND::on_actionImport_Frame_triggered()
{
    // TODO: adding individual frame

}

void VisualizeND::on_actionExport_triggered()
{
    QFile file("out.csv");
    if(file.open(QFile::WriteOnly | QFile::Truncate)){
        QTextStream stream(&file);
        stream << "Frames" << "," << "Ave Temp" << "," << "Ave Hight" << endl;
        unsigned int i = 0;
        for(list<InstanceCloud<PointType> >::iterator it = frames_.begin(); it != frames_.end(); it++){
            double ave_temp = cv::mean(it->getCloudTempCV()).val[0];
            stream << QString::number(i) << "," << QString::number(ave_temp) << "," << "0.0" << endl;
            i++;
        }
        stream.flush();
    }

    file.close();
}

void VisualizeND::on_actionExit_triggered()
{
    this->close();
}

void VisualizeND::on_actionNext_triggered()
{
    FrameIterator frameIt = this->getFrameIterator();
    frameIt++;
    if(frameIndx_ < nFrames_ - 1){
        frameIt_++;
        frameIndx_++;
        updateView();
    }

}

void VisualizeND::on_actionPrevious_triggered()
{
    FrameIterator frameIt = this->getFrameIterator();
    frameIt--;
    if(frameIndx_ > 0){
        frameIt_--;
        frameIndx_--;
        updateView();
    }
}

void VisualizeND::on_frameSlider_valueChanged(int value)
{
    int steps = value - frameIndx_;
    if(steps > 0){
        for(int i = 0; i < steps; i++){
            frameIt_++;
            frameIndx_++;
        }
    }
    else if(steps < 0){
        for(int i = 0; i < abs(steps); i++){
            frameIt_--;
            frameIndx_--;
        }

    }
    updateView();

}

void VisualizeND::on_actionPlay_triggered()
{
    play_ = true;
    while((frameIndx_ < nFrames_ - 1) && play_){
        on_actionNext_triggered();

        if(ui_->action_Rate_0_25->isChecked())
            rate_ = 0.25;
        else if(ui_->action_Rate_0_5->isChecked())
            rate_ = 0.5;
        else if(ui_->action_Rate_1_0->isChecked())
            rate_= 1.0;
        else if(ui_->action_Rate_2_0->isChecked())
            rate_= 2.0;
        else if(ui_->action_Rate_4_0->isChecked())
            rate_ = 4.0;

        sleepFor(500 / rate_);

    }
    play_ = false;

}

void VisualizeND::on_actionPause_triggered()
{
    play_ = false;

}


void VisualizeND::on_actionTop_triggered()
{
    double maxZ = max_pt_[2];
    viewer_->setCameraPosition(0, 0, 10*maxZ, 0, 0, -1);
    ui_->qvtkWidget->update();
}

void VisualizeND::on_actionBottom_triggered()
{
    double minZ = min_pt_[2];
    viewer_->setCameraPosition(0,0, 10 * minZ, 0, 0, 1);
    ui_->qvtkWidget->update();
}

void VisualizeND::on_actionLeft_triggered()
{
    double maxY = max_pt_[1];
    viewer_->setCameraPosition(0, 10 * maxY, 0, 0, -1, 0);
    ui_->qvtkWidget->update();
}

void VisualizeND::on_actionRight_triggered()
{
    double minY = min_pt_[1];
    viewer_->setCameraPosition(0, 10 * minY, 0, 0, 1, 0);
    ui_->qvtkWidget->update();
}

void VisualizeND::on_actionFront_triggered()
{
    double maxX = max_pt_[0];
    viewer_->setCameraPosition(10 * maxX, 0, 0, -1, 0, 0);
    ui_->qvtkWidget->update();
}

void VisualizeND::on_actionBack_triggered()
{
    double minX = min_pt_[0];
    viewer_->setCameraPosition(10 * minX, 0, 0, 1, 0, 0);
    ui_->qvtkWidget->update();
}

void VisualizeND::on_actionPerspective_triggered()
{
    double maxX = max_pt_[0];
    double maxY = max_pt_[1];
    double maxZ = max_pt_[2];
    viewer_->setCameraPosition(5 * maxX, 5 * maxY, 5 * maxZ, -1, -1, -1);
    ui_->qvtkWidget->update();
}

void VisualizeND::on_actionRGB_triggered()
{
    updateView();
}

void VisualizeND::on_actionTemperature_triggered()
{
    updateView();
}

void VisualizeND::on_actionAutumn_triggered()
{
    colormap_ = cv::COLORMAP_AUTUMN;
    updateColorBar();
    updateView();
}

void VisualizeND::on_actionBone_triggered()
{
    colormap_ = cv::COLORMAP_BONE;
    updateColorBar();
    updateView();
}

void VisualizeND::on_actionJet_triggered()
{
    colormap_ = cv::COLORMAP_JET;
    updateColorBar();
    updateView();
}

void VisualizeND::on_actionWinter_triggered()
{
    colormap_ = cv::COLORMAP_WINTER;
    updateColorBar();
    updateView();
}

void VisualizeND::on_actionRainbow_triggered()
{
    colormap_ = cv::COLORMAP_RAINBOW;
    updateColorBar();
    updateView();
}

void VisualizeND::on_actionOcean_triggered()
{
    colormap_ = cv::COLORMAP_OCEAN;
    updateColorBar();
    updateView();
}

void VisualizeND::on_actionSummer_triggered()
{
    colormap_ = cv::COLORMAP_SUMMER;
    updateColorBar();
    updateView();
}

void VisualizeND::on_actionSpring_triggered()
{
    colormap_ = cv::COLORMAP_SPRING;
    updateColorBar();
    updateView();
}

void VisualizeND::on_actionCool_triggered()
{
    colormap_ = cv::COLORMAP_COOL;
    updateColorBar();
    updateView();
}

void VisualizeND::on_actionHSV_triggered()
{
    colormap_ = cv::COLORMAP_HSV;
    updateColorBar();
    updateView();
}

void VisualizeND::on_actionPink_triggered()
{
    colormap_ = cv::COLORMAP_PINK;
    updateColorBar();
    updateView();
}

void VisualizeND::on_actionHot_triggered()
{
    colormap_ = cv::COLORMAP_HOT;
    updateColorBar();
    updateView();
}

void VisualizeND::on_actionColor_Bar_triggered(bool checked)
{
    if(checked){
        ui_->labelColorMap->show();
        ui_->labelMaxTemp->show();
        ui_->labelMinTemp->show();
        ui_->lineEditMaxTemp->show();
        ui_->lineEditMinTemp->show();
    }
    else{
        ui_->labelColorMap->hide();
        ui_->labelMaxTemp->hide();
        ui_->labelMinTemp->hide();
        ui_->lineEditMaxTemp->hide();
        ui_->lineEditMinTemp->hide();
    }
}

void VisualizeND::on_lineEditMaxTemp_editingFinished()
{
    int new_max = ui_->lineEditMaxTemp->text().toInt();
    min_max_temp_[frameIndx_].second = new_max;
    updateView();
}

void VisualizeND::on_lineEditMinTemp_editingFinished()
{
    int new_min = ui_->lineEditMinTemp->text().toInt();
    min_max_temp_[frameIndx_].first = new_min;
    updateView();

}

void VisualizeND::on_actionSelect_Points_triggered(bool checked)
{
    if(checked){
        framesSelectedPoints_.clear();
        framesSelectedPoints_.resize(frames_.size());
        for(unsigned int i = 0; i < frames_.size(); i++){
            pcl::PointCloud<PointType>::Ptr cloudPtr(new pcl::PointCloud<PointType>);
            framesSelectedPoints_[i] = cloudPtr;
        }

    }
    else{
        framesSelectedPoints_.clear();
        framesSelectedPoints_.resize(frames_.size());
    }
    updateView();
}

void VisualizeND::on_actionAlign_triggered()
{
    if(ui_->actionSelect_Points->isChecked()){
        list< InstanceCloud<PointType> >::iterator frameIt = frames_.begin();
        frameIt++;
        for(unsigned int i = 1; i < frames_.size(); i++){
            pcl::registration::TransformationEstimationSVD<PointType,PointType> TESVD;
            pcl::registration::TransformationEstimationSVD<PointType, PointType>::Matrix4 tf;
            TESVD.estimateRigidTransformation(*framesSelectedPoints_[i],*framesSelectedPoints_[0], tf);

            pcl::PointCloud<PointType>::Ptr cloud = frameIt->getCloudPtr();
            pcl::transformPointCloud(*cloud, *cloud, tf);
            frameIt++;
        }
    }
    else{
        QMessageBox::warning(this, "Error", "No points is selected");
    }


}

void VisualizeND::on_actionManual_triggered()
{
    X = 0;
    Y = 0;
    Z = 0;
    Nx = 0;
    Ny = 0;
    Nz = 1.0;
    plane_dialog_ = new PlaneDialog(this);
    plane_dialog_->show();

}

void VisualizeND::on_actionMeasure_triggered()
{

    if(ui_->actionSelect_Points->isChecked()){
        list< InstanceCloud<PointType> >::iterator frameIt = frames_.begin();
        for(unsigned int i = 0; i < frames_.size(); i++){
            for(unsigned int j = 0; j < framesSelectedPoints_[i]->points.size(); j++){
                PointType p = framesSelectedPoints_[i]->points[j];
                // measure ave height and temperature
                float radius = 100.0;   // mm
                vector<int> pointIdxRadiusSearch;
                pointIdxRadiusSearch = frameIt->searchPointByRadius(p, radius);
                float sumZ = 0;
                float sumT = 0;
                for(size_t k = 0; k < pointIdxRadiusSearch.size(); ++k){
                    PointType np = frameIt->getCloudPtr()->points[pointIdxRadiusSearch[k]];
                    sumZ += np.z;
                    if(tempAvailable_){
                        sumT += frameIt->getCloudTempCV().at<double>(pointIdxRadiusSearch[k],0);
                    }
                }
                float aveZ = sumZ / pointIdxRadiusSearch.size();
                float aveTemp = sumT / pointIdxRadiusSearch.size();
                cout << "Point " << j+1 << ": Ave Z = " << aveZ << " (mm) , Ave Temp = " << aveTemp << " (C)" << endl;
            }
            frameIt++;
        }

    }
    else{
        QMessageBox::warning(this, "Error", "No points is selected");
    }

}
