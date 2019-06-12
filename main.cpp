//----------------------------------------------//
//           N-D Visualization GUI              //
//                                              //
//        Copyright (c) ViGIR-Lab 2019          //
//          Written by Ali Shfiekhani           //
//----------------------------------------------//

#include "visualize_nd.h"

#include <QApplication>
#include <QMainWindow>

int main(int argc, char **argv){
    QApplication VisNDGUI(argc, argv);
    
    VisualizeND vis;
    vis.show();

    return VisNDGUI.exec();
}
