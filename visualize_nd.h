//----------------------------------------------//
//           N-D Visualization GUI              //
//                                              //
//        Copyright (c) ViGIR-Lab 2019          //
//          Written by Ali Shfiekhani           //
//----------------------------------------------//

#ifndef VISUALIZE_5D_H
#define VISUALIZE_5D_H

#define BOOST_PREDEF_PLATFORM_H
#define BOOST_PREDEF_OS_H

#pragma once

#include <boost/filesystem.hpp>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>

// STD
#include <vector>
#include <utility>

// QT
#include <QMainWindow>
#include <QFileDialog>
#include <QtCore/QtPlugin>
#include <QCoreApplication>
#include <QThread>
#include <QDateTime>
#include <QActionGroup>
#include <QDir>
#include <QMessageBox>
#include <QFileInfo>
#include <QTextStream>


// Point Cloud Library (PCL)
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/common/transforms.h>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>


// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

#include "instance_cloud.h"
#include "ui_visualize_nd.h"
#include "ui_planedialog.h"
#include "planedialog.h"

using namespace std;

typedef pcl::PointXYZRGBA PointType;
typedef typename list< InstanceCloud<PointType> >::iterator FrameIterator;

class PlaneDialog;

namespace Ui{
    class VisualizeND;
}

class VisualizeND : public QMainWindow{
    Q_OBJECT

    public:
    /**
     * Constructor
     */
    explicit VisualizeND(QWidget *parent = 0);

    ~VisualizeND();

    /**
     * Adds an instance of cloud (one frame) to the list of frames 
     * @param instance_cloud 
     */ 
    void addInstaceCloud(InstanceCloud<PointType> instance_cloud);

    /**
     * Loads all frames from folder based on alphabetic order
     * @param files_path 
     */
    bool loadFrames(string files_path);

    /**
     * Returns the the viewer
     */
    pcl::visualization::PCLVisualizer::Ptr getViewer();

    /**
     *Returns current frame iterator  
     */
    FrameIterator getFrameIterator();

    /**
     * Returns total number of frames loaded 
     */
    unsigned int getNumFrames();

    static void pointPickingEventOccurred(const pcl::visualization::PointPickingEvent &event, void* visClassVoidPtr);

    /**
     * Keyboard Callback: 
     * "t" to toggle between RGB and Temperature
     * "Left"/"Down" to change the visualized cloud to previous frame  
     * "Right"/"Up" to change the visualized cloud to next frame 
     */
    static void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* visClassVoidPtr);
    
    /**
     * Runs the visualization
     */
    void init();
    void updateView();
    void updateColorBar();

    public Q_SLOTS:


private slots:
    void on_actionOpen_triggered();

    void on_actionImport_Frame_triggered();

    void on_actionExport_triggered();

    void on_actionExit_triggered();

    void on_actionNext_triggered();

    void on_actionPrevious_triggered();

    void on_frameSlider_valueChanged(int value);

    void on_actionPlay_triggered();

    void on_actionPause_triggered();

    void on_actionTop_triggered();

    void on_actionBottom_triggered();

    void on_actionLeft_triggered();

    void on_actionRight_triggered();

    void on_actionFront_triggered();

    void on_actionBack_triggered();

    void on_actionPerspective_triggered();

    void on_actionRGB_triggered();

    void on_actionTemperature_triggered();

    void on_actionAutumn_triggered();

    void on_actionBone_triggered();

    void on_actionJet_triggered();

    void on_actionWinter_triggered();

    void on_actionRainbow_triggered();

    void on_actionOcean_triggered();

    void on_actionSummer_triggered();

    void on_actionSpring_triggered();

    void on_actionCool_triggered();

    void on_actionHSV_triggered();

    void on_actionPink_triggered();

    void on_actionHot_triggered();

    void on_actionColor_Bar_triggered(bool checked);

    void on_lineEditMaxTemp_editingFinished();

    void on_lineEditMinTemp_editingFinished();

    void on_actionSelect_Points_triggered(bool checked);

    void on_actionAlign_triggered();

    void on_actionManual_triggered();

    void on_actionMeasure_triggered();

private:
    void sleepFor(size_t milisec);

    friend class PlaneDialog;

private:
    list< InstanceCloud<PointType> > frames_;
    vector< pcl::PointCloud<PointType>::Ptr > framesSelectedPoints_;
    FrameIterator frameIt_;
    unsigned int frameIndx_;
    pcl::VoxelGrid<PointType> grid_;
    float gridSize_;
    Eigen::Vector4f min_pt_;
    Eigen::Vector4f max_pt_;
    vector< pair<double, double> > min_max_temp_;
    int colormap_;
    cv::Mat colorBar_;
    string filesPath_;
    bool tempAvailable_;
    bool tfAvailable_;
    pcl::visualization::PCLVisualizer::Ptr viewer_;
    pcl::PointCloud<PointType>::Ptr clickedPoints_;
    unsigned int nFrames_;

    Ui::VisualizeND *ui_;
    PlaneDialog *plane_dialog_;

    bool play_;
    double rate_;


    double X;
    double Y;
    double Z;
    double Nx;
    double Ny;
    double Nz;

};


#endif
