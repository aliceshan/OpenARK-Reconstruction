// C++ standard
#include <thread>
#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>
#include <cstring>
#ifdef WINDOWS
    #include <direct.h>
    #define GetCurrentDir _getcwd
#else
    #include <unistd.h>
    #define GetCurrentDir getcwd
#endif
#include <unordered_map>

// PCL libraries
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/3rdparty/poisson4/vector.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
//#include <pcl/io/pcd_io.h>

// VTK libraries
#include <vtkQuadricClustering.h>
#include <vtkPolyData.h>
#include <vtkPLYReader.h>

// OpenCV libraries
#include <opencv2/opencv.hpp>

#include "tinyply.h"
#include "halfEdgeMesh.h"

using namespace tinyply;
using namespace CGL;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr;
pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr kdtree;
//HalfedgeMesh mesh;
std::vector<Vector3D> verts;
std::vector<std::vector<Index>> polygons;
std::vector<std::vector<Vector2D>> text;

double triangleBuffer;

void removePlane(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        pcl::ModelCoefficients::Ptr coefficients,
        pcl::PointIndices::Ptr inliers
) {
    // RANSAC plane fitting.
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.07);
    seg.setMaxIterations(100);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
}

pcl::poisson::Vector<double> project(
        pcl::poisson::Vector<double> normal, pcl::poisson::Vector<double> orig, pcl::poisson::Vector<double> pt
) {
    pcl::poisson::Vector<double> v = pt - orig;
    double dist = v.Dot(normal);
    return pt - normal * dist;
}

std::string pointToString(pcl::PointXYZ pt) {
    return (std::to_string(pt.x) + std::to_string(pt.y) + std::to_string(pt.z));
}

void vtkPolyData2Vectors(vtkSmartPointer<vtkPolyData> mesh) {
    verts.reserve(mesh->GetNumberOfPoints());
    polygons.reserve(mesh->GetNumberOfPolys());
    for (int i = 0; i < mesh->GetNumberOfPoints(); ++i) {
        verts.push_back(Vector3D(mesh->GetPoint(i)[0], mesh->GetPoint(i)[1], mesh->GetPoint(i)[2]));
    }

    for (int j = 0; j < mesh->GetNumberOfPolys(); ++j) {
        std::vector<Index> f;
        f.reserve(3);
        f.push_back((Index) mesh->GetCell(j)->GetPointId(0));
        f.push_back((Index) mesh->GetCell(j)->GetPointId(1));
        f.push_back((Index) mesh->GetCell(j)->GetPointId(2));
        polygons.push_back(f);
    }
}

void decimateAndProject(std::string inputFileName) {
    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
    reader->SetFileName(inputFileName.c_str());
    reader->Update();
    vtkSmartPointer<vtkPolyData> inputPolyData = reader->GetOutput();

    std::cout << "Before decimation" << std::endl << "------------" << std::endl;
    std::cout << "There are " << inputPolyData->GetNumberOfPoints() << " points." << std::endl;
    std::cout << "There are " << inputPolyData->GetNumberOfPolys() << " polygons." << std::endl;

    vtkSmartPointer<vtkQuadricClustering> decimate = vtkSmartPointer<vtkQuadricClustering>::New();
#if VTK_MAJOR_VERSION <= 5
    decimate->SetInputConnection(inputPolyData2->GetProducerPort());
#else
    decimate->SetInputData(inputPolyData);
#endif
    //decimate->PreserveTopologyOn();
    //decimate->SplittingOn();
    //decimate->BoundaryVertexDeletionOn();
    //decimate->SetMaximumError(0.00001);

    int targetFaces = (int) inputPolyData->GetNumberOfPolys() / 10;
    decimate->SetNumberOfDivisions(cbrt(targetFaces), cbrt(targetFaces), cbrt(targetFaces));
    decimate->Update();

    vtkSmartPointer<vtkPolyData> decimated =
            vtkSmartPointer<vtkPolyData>::New();
    decimated->ShallowCopy(decimate->GetOutput());

    std::cout << "After decimation" << std::endl << "------------" << std::endl;

    std::cout << "There are " << decimated->GetNumberOfPoints() << " points." << std::endl;
    std::cout << "There are " << decimated->GetNumberOfPolys() << " polygons." << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_ptr->width = decimated->GetNumberOfPoints();
    cloud_ptr->height = 1;
    cloud_ptr->is_dense = false;
    cloud_ptr->points.resize(cloud_ptr->width * cloud_ptr->height);
    std::unordered_map<std::string, int> map;
    for (size_t i = 0; i < cloud_ptr->points.size(); ++i) {
        double p[3];
        decimated->GetPoint(i, p);
        cloud_ptr->points[i].x = p[0];
        cloud_ptr->points[i].y = p[1];
        cloud_ptr->points[i].z = p[2];
        map[pointToString(cloud_ptr->points[i])] = i;
    }
//
//    pcl::ExtractIndices<pcl::PointXYZ> extract;
//    while (true) {
//        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//        removePlane(cloud_ptr, coefficients, inliers);
//        if (inliers->indices.size() < 3000)
//        {
//            break;
//        } else {
//            pcl::poisson::Vector<double> v(3);
//            v[0] = coefficients->values[0];
//            v[1] = coefficients->values[1];
//            v[2] = coefficients->values[2];
//            v.Normalize();
//            pcl::poisson::Vector<double> orig(3);
//            orig[0] = 0.0;
//            orig[1] = 0.0;
//            orig[2] = -coefficients->values[3] / coefficients->values[2];
//            for (size_t i = 0; i < inliers->indices.size(); ++i) {
//                pcl::poisson::Vector<double> pt(3);
//                pt[0] = cloud_ptr->points[inliers->indices[i]].x;
//                pt[1] = cloud_ptr->points[inliers->indices[i]].y;
//                pt[2] = cloud_ptr->points[inliers->indices[i]].z;
//                pcl::poisson::Vector<double> projected = project(v, orig, pt);
//                // cloud_ptr->points[inliers->indices[i]].x = projected[0];
//                // cloud_ptr->points[inliers->indices[i]].y = projected[1];
//                // cloud_ptr->points[inliers->indices[i]].z = projected[2];
//                decimated->GetPoints()->SetPoint(map[pointToString(cloud_ptr->points[inliers->indices[i]])],
//                                                 projected[0], projected[1], projected[2]);
//
//            }
//            extract.setInputCloud (cloud_ptr);
//            extract.setIndices (inliers);
//            extract.setNegative (true);
//            extract.filter (*cloud_ptr);
//        }
//    }
    vtkPolyData2Vectors(decimated);
}

void write_ply_example(const std::string & filename, const std::string &textFileName)
{
    std::vector<float> vertices;
    vertices.reserve(3 * verts.size());
    std::vector<int32_t> vertexIndicies;
    vertexIndicies.reserve(3 * polygons.size());
    std::vector<float> faceTexcoords;
    faceTexcoords.reserve(6 * text.size());

    // Per-vertex elements
    for (int i = 0; i < verts.size(); ++i) {
        vertices.push_back(verts[i].x);
        vertices.push_back(verts[i].y);
        vertices.push_back(verts[i].z);
    }

    // Per-face elements
    for (int j = 0; j < polygons.size(); ++j) {
        vertexIndicies.push_back(polygons[j][0]);
        vertexIndicies.push_back(polygons[j][1]);
        vertexIndicies.push_back(polygons[j][2]);
    }

    for (int k = 0; k < text.size(); ++k) {
        faceTexcoords.push_back(text[k][0].x);
        faceTexcoords.push_back(text[k][0].y);
        faceTexcoords.push_back(text[k][1].x);
        faceTexcoords.push_back(text[k][1].y);
        faceTexcoords.push_back(text[k][2].x);
        faceTexcoords.push_back(text[k][2].y);
    }

    // Tinyply does not perform any file i/o internally
    std::filebuf fb;
    fb.open(filename, std::ios::out | std::ios::binary);
    std::ostream outputStream(&fb);

    PlyFile exampleOutFile;

    exampleOutFile.add_properties_to_element("vertex", { "x", "y", "z" }, Type::FLOAT32, vertices.size(), reinterpret_cast<uint8_t*>(vertices.data()), Type::INVALID, 0);
    //exampleOutFile.add_properties_to_element("vertex", { "nx", "ny", "nz" }, Type::FLOAT32, verts.size(), reinterpret_cast<uint8_t*>(norms.data()), Type::INVALID, 0);
    //exampleOutFile.add_properties_to_element("vertex", { "red", "green", "blue", "alpha" }, Type::UINT8, verts.size(), reinterpret_cast<uint8_t*>(colors.data()), Type::INVALID, 0);

    exampleOutFile.add_properties_to_element("face", { "vertex_indices" }, Type::INT32, vertexIndicies.size(), reinterpret_cast<uint8_t*>(vertexIndicies.data()), Type::UINT8, 3);
    exampleOutFile.add_properties_to_element("face", { "texcoord" }, Type::FLOAT32, faceTexcoords.size(), reinterpret_cast<uint8_t*>(faceTexcoords.data()), Type::UINT8, 6);

    exampleOutFile.get_comments().push_back("TextureFile " + textFileName);
    exampleOutFile.write(outputStream, false);

    fb.close();
}

void read_ply_file(const std::string & filename)
{
    try {
        // Read the file and create a std::istringstream suitable
        // for the lib -- tinyply does not perform any file i/o.
        std::ifstream ss(filename, std::ios::binary);

        if (ss.fail()) {
            throw std::runtime_error("failed to open " + filename);
        }

        PlyFile file;

        file.parse_header(ss);

        std::cout << "================================================================\n";

        for (auto c : file.get_comments()) std::cout << "Comment: " << c << std::endl;

        for (auto e : file.get_elements()) {
            std::cout << "element - " << e.name << " (" << e.size << ")" << std::endl;
            for (auto p : e.properties) {
                std::cout << "\tproperty - " << p.name << " (" << tinyply::PropertyTable[p.propertyType].str << ")"
                          << std::endl;
            }
        }

        std::cout << "================================================================\n";
        // Tinyply 2.0 treats incoming data as untyped byte buffers. It's now
        // up to users to treat this data as they wish. See below for examples.
        std::shared_ptr<PlyData> vertices, normals, colors, faces, texcoords;

        // The header information can be used to programmatically extract properties on elements
        // known to exist in the file header prior to reading the data. For brevity of this sample, properties 
        // like vertex position are hard-coded: 
        try { vertices = file.request_properties_from_element("vertex", {"x", "y", "z"}); }
        catch (const std::exception &e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

        try { normals = file.request_properties_from_element("vertex", {"nx", "ny", "nz"}); }
        catch (const std::exception &e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

        try { colors = file.request_properties_from_element("vertex", {"red", "green", "blue", "alpha"}); }
        catch (const std::exception &e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

        try { faces = file.request_properties_from_element("face", {"vertex_indices"}); }
        catch (const std::exception &e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

        try { texcoords = file.request_properties_from_element("face", {"texcoord"}); }
        catch (const std::exception &e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

        file.read(ss);

        // Good place to put a breakpoint!
        if (vertices) std::cout << "\tRead " << vertices->count << " total vertices " << std::endl;
        if (normals) std::cout << "\tRead " << normals->count << " total vertex normals " << std::endl;
        if (colors) std::cout << "\tRead " << colors->count << " total vertex colors " << std::endl;
        if (faces) std::cout << "\tRead " << faces->count << " total faces (triangles) " << std::endl;
        if (texcoords) std::cout << "\tRead " << texcoords->count << " total texcoords " << std::endl;

        // Example: type 'conversion' to your own native types - Option A
//        {
        const size_t numVerticesBytes = vertices->buffer.size_bytes();
        struct float3 {
            float x, y, z;
        };
        std::vector<float3> verts_tmp(vertices->count);
        std::memcpy(verts_tmp.data(), vertices->buffer.get(), numVerticesBytes);
        verts.reserve(vertices->count);
        for (int i = 0; i < verts_tmp.size(); ++i) {
//            verts.push_back(Vector3D(verts_tmp[i].x - 1, verts_tmp[i].y - 1, verts_tmp[i].z + 1.5)); // test for cube
            verts.push_back(Vector3D(verts_tmp[i].x, verts_tmp[i].y, verts_tmp[i].z));
        }
//    }

        const size_t numFaceBytes = faces->buffer.size_bytes();
        struct uint3 {
            uint32_t x, y, z;
        };
        std::vector<uint3> face_tmp(faces->count);
        std::memcpy(face_tmp.data(), faces->buffer.get(), numFaceBytes);
        polygons.reserve(vertices->count);
        for (int i = 0; i < face_tmp.size(); ++i) {
            std::vector<Index> f;
            f.reserve(3);
            f.push_back((Index) face_tmp[i].x);
            f.push_back((Index) face_tmp[i].y);
            f.push_back((Index) face_tmp[i].z);
            polygons.push_back(f);
        }

//        // Example: type 'conversion' to your own native types - Option B
//        {
//            const size_t numVerticesBytes = vertices->buffer.size_bytes();
//            struct float3 { float x, y, z; };
//            struct double3 { double x, y, z; };
//
//            std::vector<float3> verts_floats;
//            std::vector<double3> verts_doubles;
//
//            if (vertices->t == tinyply::Type::FLOAT32) { /* as floats ... */ }
//            if (vertices->t == tinyply::Type::FLOAT64) { /* as doubles ... */ }
//        }
    }
    catch (const std::exception & e)
    {
        std::cerr << "Caught tinyply exception: " << e.what() << std::endl;
    }
}

void read_color_cloud(std::string fileName) {
    pcl::io::loadPLYFile(fileName, *cloud_ptr);
    kdtree->setInputCloud (cloud_ptr);
    kdtree->setEpsilon(0.03);
}


std::vector<Vector2D> getTriangleCoords(Vector2D anchor, double edgeLength) {
    std::vector<Vector2D> tri;
    tri.reserve(3);
    tri.push_back(anchor);
    tri.push_back(anchor + Vector2D(edgeLength, 0));
    tri.push_back(anchor + Vector2D(0, edgeLength));
    return tri;
}

void assignTextCoord() {
    double triangleProportion = 0.95;
    size_t numFaces = polygons.size();
    text.reserve(numFaces);
    size_t numTrianglePerSide = ceil(sqrt(numFaces));
    double triangleSideLength = 0.99 / numTrianglePerSide;
    triangleBuffer = (1.0 - triangleProportion) * triangleSideLength;
    triangleSideLength = triangleProportion * triangleSideLength;
    size_t x_counter = 0;
    size_t y_counter = 0;
    for (size_t i = 0; i < numFaces; ++i) {
        if (x_counter > numTrianglePerSide - 1) {
            x_counter = 0;
            y_counter = y_counter + 1;
        }
        Vector2D anchor(x_counter * (triangleSideLength + triangleBuffer),
                        y_counter * (triangleSideLength + triangleBuffer));
        text.push_back(getTriangleCoords(anchor, triangleSideLength));
        x_counter = x_counter + 1;
    }
}

Vector3D getBarycentric(const Vector2D &query, const Vector2D &a, const Vector2D &b, const Vector2D &c) {
    double bary_a = (-(query.x - b.x) * (c.y - b.y) + (query.y - b.y) * (c.x - b.x)) /
                (-(a.x - b.x) * (c.y - b.y) + (a.y - b.y) * (c.x - b.x));
    double bary_b = (-(query.x - c.x) * (a.y - c.y) + (query.y - c.y) * (a.x - c.x)) /
                (-(b.x - c.x) * (a.y - c.y) + (b.y - c.y) * (a.x - c.x));
    double bary_c = (-(query.x - a.x) * (b.y - a.y) + (query.y - a.y) * (b.x - a.x)) /
                (-(c.x - a.x) * (b.y - a.y) + (c.y - a.y) * (b.x - a.x));
    return Vector3D(bary_a, bary_b, bary_c);
}

Vector3D getQueryPointIn3D(const Vector3D &bary, const Vector3D &a, const Vector3D &b, const Vector3D &c) {
    double x = a.x * bary[0] + b.x * bary[1] + c.x * bary[2];
    double y = a.y * bary[0] + b.y * bary[1] + c.y * bary[2];
    double z = a.z * bary[0] + b.z * bary[1] + c.z * bary[2];
    return Vector3D(x, y, z);
}

cv::Vec3b getNearestColor(const Vector3D &query, int K) {
    pcl::PointXYZRGB searchPoint;
    searchPoint.x = query.x;
    searchPoint.y = query.y;
    searchPoint.z = query.z;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    //float radius = 0.05f;
    Vector3D color(0.0, 0.0, 0.0);
    if (kdtree->nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
        for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i) {
            color = color + Vector3D(cloud_ptr->points[ pointIdxNKNSearch[i] ].b,
                                     cloud_ptr->points[ pointIdxNKNSearch[i] ].g,
                                     cloud_ptr->points[ pointIdxNKNSearch[i] ].r);
        }
    } else {
        std::cerr << "Can't find any point near query, abort" << std::endl;
        exit(1);
//        cv::Vec3b black(0, 0, 0);
//        return black;
    }
    color = color / pointIdxNKNSearch.size();
    cv::Vec3b c((uchar) color.x, (uchar) color.y, (uchar) color.z);
    return c;
}

void boundaryCheck(Vector2D &point, int sideLength) {
    if (point.x < 0) {
        point.x = 0;
    }
    if (point.x > sideLength - 1) {
        point.x = sideLength - 1;
    }
    if (point.y < 0) {
        point.y = 0;
    }
    if (point.y > sideLength - 1) {
        point.y = sideLength - 1;
    }
}

cv::Mat rasterizeTexture(int sideLength, int K) {
    cv::Mat textImg(sideLength, sideLength, CV_8UC3, cv::Scalar(255, 255, 255));
    // Loop through all triangles
    for (size_t i = 0; i < text.size(); ++i) {
        Vector2D a, b, c;
        // It is assumed that it's a right triangle with the right angle on upper left corner, and
        // second point is at straight right of it and third poitn is at straight below it
        a = Vector2D(floor(text[i][0].x * sideLength), floor(text[i][0].y * sideLength));
        b = Vector2D(ceil(text[i][1].x * sideLength), floor(text[i][1].y * sideLength));
        c = Vector2D(floor(text[i][2].x * sideLength), ceil(text[i][2].y * sideLength));
        int buffer = (int) ceil(sideLength * triangleBuffer / 2);
        a = a + Vector2D(-buffer, -buffer);
//        b = b + Vector2D(buffer, -buffer);
//        c = c + Vector2D(-buffer, buffer);
        boundaryCheck(a, sideLength);
//        boundaryCheck(b, sideLength);
//        boundaryCheck(c, sideLength);
        // Loop through triangles on the texture
        for (int j = 0; j < b.x - a.x; ++j) {
            for (int k = 0; k < c.y - a.y; ++k) {
                Vector2D query(a.x + j, a.y + k);
                // Only rasterize the upper left triangle
                // Details can be seen at: https://cs184.eecs.berkeley.edu/lecture/sampling/slide_043
                Vector2D v = query - c;
                Vector2D n = Vector2D(-(b.y - c.y), (b.x - c.x));
//                if (dot(v, n) <= 0) {
                    Vector3D bary = getBarycentric(query, a, b, c);
                    Vector3D x_3d, y_3d, z_3d;
                    x_3d = verts[polygons[i][0]];
                    y_3d = verts[polygons[i][1]];
                    z_3d = verts[polygons[i][2]];
                    Vector3D query3d = getQueryPointIn3D(bary, x_3d, y_3d, z_3d);
                    textImg.at<cv::Vec3b>(sideLength - (int) query.y, (int) query.x) = getNearestColor(query3d, K);
//                    textImg.at<cv::Vec3b>(sideLength - (int) query.y, (int) query.x)[0] = 0;
//                    textImg.at<cv::Vec3b>(sideLength - (int) query.y, (int) query.x)[1] = 0;
//                    textImg.at<cv::Vec3b>(sideLength - (int) query.y, (int) query.x)[2] = (k / (c.y - a.y)) * 255;
//                }
            }
        }
    }
    return textImg;
}

std::string saveTextureToCurrDir(cv::Mat m, std::string fileName) {
    char cCurrentPath[FILENAME_MAX];
    if (!GetCurrentDir(cCurrentPath, sizeof(cCurrentPath)))
    {
        exit(1);
    }
    cCurrentPath[sizeof(cCurrentPath) - 1] = '\0';
    std::string fullPath = std::string(cCurrentPath) + "/" + fileName;
    cv::imwrite(fullPath, m);
    return fullPath;
}

int main(int argc, char *argv[])
{
    if(argc!=3) {
        std::cout << "Usage: ./colorizer [Input model path] [texture resolution]"<<std::endl;
        return 0;
    }
    std::string denseCloudName = argv[1];
    std::string textureImgName = string(argv[1]).substr(0,string(argv[1]).find("."))+"_tex.png";
    int size = atoi(argv[2]);
    std::string finalOutputName = string(argv[1]).substr(0,string(argv[1]).find("."))+"_simplified.ply";

    cloud_ptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    kdtree = pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZRGB>());

    decimateAndProject(denseCloudName);
    read_color_cloud(denseCloudName);
//    read_ply_file(unTexturedPlyName);
    assignTextCoord();
    cv::Mat texture = rasterizeTexture(size, 3);
    std::string fullPath = saveTextureToCurrDir(texture, textureImgName);
    write_ply_example(finalOutputName, fullPath);

//    pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/will/Downloads/out2.pcd", *cloud_ptr);
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//    viewer->setBackgroundColor(0, 0, 0);
//    viewer->addPointCloud<pcl::PointXYZRGB> (cloud_ptr, "sample cloud");
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
//    viewer->addCoordinateSystem (1.0);
//    viewer->initCameraParameters ();
//    while (!viewer->wasStopped ())
//    {
//        viewer->spinOnce ();
//    }

    cloud_ptr.reset();
    kdtree.reset();
    return 0;
}
