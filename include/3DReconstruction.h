//API

#ifndef 3D_RECON_H
#define 3D_RECON_H

#include <iostream>
#include <algorithm>
#include <thread>
#include <sstream>

#include <opencv2/opencv.hpp>

#include <Colorizer.h>

//VTK libraries
#include <vtkQuadricClustering.h>
#include <vtkPolyData.h>
#include <vtkPLYReader.h>


namespace ark{

    struct Vertex {
        float x;
        float y;
        float z;
        unsigned char r;
        unsigned char g;
        unsigned char b;

        Vertex() {}
        Vertex(float xi, float yi, float zi) : x(xi), y(yi), z(zi) {}
        Vertex(float xi, float yi, float zi, unsigned char ri, unsigned char gi, unsigned char bi){
            x = xi;
            y = yi;
            z = zi;
            r = ri;
            g = gi;
            b = bi;
        }
    };

    struct Face {
        int vIdx[3];
        Face() {}
        Face(int a, int b, int c){
            vIdx[0] = a;
            vIdx[0] = b;
            vIdx[0] = c;
        }
    };

    class 3DRecon{
    public:
        3DRecon();

        void Initialize(std::string strSettingsFile);

        void Update(const cv::Mat &imRGB, const cv::Mat &imD, const cv::Mat &Twc);

        void GetModel(); //save the current mesh model on TSDF

        void Save2file(std::string filename);

        void Render();

        void LoadFromFile(std::string inputFileName); //ply file using VTK library, update mesh model

        //haven't implemented the 2 below
        void Colorize(int resolution, std::string textureImageName, std::string finalOutputName);

        void Simplify();

        void ~3DRecon(){}

    private:
        std::vector<Vertex> myVertices; //mesh model
        std::vector<Face> myFaces; 
    }
}


#endif