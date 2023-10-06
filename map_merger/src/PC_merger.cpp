#include "../nanoflann.hpp"  // Include the nanoflann library for KD-tree operations
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>  // Include Eigen library for linear algebra operations
#include <Eigen/SVD>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <chrono>
#include <string>
#include <fstream>
#include <vector>

using namespace Eigen;
using namespace nanoflann;

// Define a struct named MatrixCollection to hold matrices.
struct MatrixCollection 
{
    Matrix4Xd mat;   // A 4xN matrix
    Matrix3Xd mat_C; // A 3xN matrix
};

// Function to read PLY file and convert it to a MatrixCollection.
MatrixCollection PlytoXYZRGB(const std::string& inputFileName)
{
    // Read the PLY file
    std::ifstream inputFile(inputFileName, std::ios::binary);
    if (!inputFile.is_open()) {
        std::cerr << "Error: Unable to open the input file." << std::endl;
    }

    // Read the header to determine the number of points
    std::string line;
    int lineCounter = 0;
    while (std::getline(inputFile, line)) {
        if (line.find("element vertex") != std::string::npos) {
            lineCounter = std::stoi(line.substr(line.find_last_of(' ') + 1));
        }
        if (line == "end_header") {
            std::cout << line << std::endl;
            break;
        }
    }
    MatrixCollection tot_matrix;
    Matrix4Xd data_PC(4, lineCounter);
    Matrix3Xd data_PC_color(3, lineCounter);
    data_PC.setZero();
    data_PC_color.setZero();

    int count = 0;
    int index;
    while (count < lineCounter) 
    {
        std::getline(inputFile, line);
        index = 0;
        for (int i = 0; i < 6; i++) {
            std::string number = line.substr(index, line.find(" ", index + 1) - index + 1);
            double num = strtod((char*)&number[0], NULL);
            if (i < 3) {
                data_PC(i, count) = num;
            }
            else {
                data_PC_color(i-3, count) = num;
            }
            index = line.find(" ", index + 1);
        }
        data_PC(3, count) = 1;
        count++;
    }
    tot_matrix.mat = data_PC;
    tot_matrix.mat_C = data_PC_color;
    return tot_matrix;
}

// Function to merge two MatrixCollections.
MatrixCollection mergePC(const MatrixCollection& PC_1, const MatrixCollection& PC_2)
{
    MatrixCollection new_PC;
    Matrix4Xd PC(4, PC_1.mat.cols() + PC_2.mat.cols());
    Matrix3Xd PC_C(3, PC_1.mat_C.cols() + PC_2.mat_C.cols());
    PC << PC_1.mat, PC_2.mat;
    PC_C << PC_1.mat_C, PC_2.mat_C;

    new_PC.mat = PC;
    new_PC.mat_C = PC_C;
    return new_PC;
}

// Function to save a point cloud in PLY format.
void savePC(const Matrix4Xd& Pcl, const Matrix3Xd& Pcl_C, const std::string& output_file)
{
    std::stringstream out3d;
    out3d << output_file << ".ply";
    std::ofstream outfile(out3d.str());

    // Write PLY header
    outfile << "ply" << std::endl;
    outfile << "format ascii 1.0" << std::endl;
    outfile << "element vertex " << Pcl.cols() << std::endl;
    outfile << "property float x" << std::endl;
    outfile << "property float y" << std::endl;
    outfile << "property float z" << std::endl;
    outfile << "property uchar red" << std::endl;
    outfile << "property uchar green" << std::endl;
    outfile << "property uchar blue" << std::endl;
    outfile << "end_header" << std::endl;		

    // Write point cloud data
    for (int i = 0; i < Pcl.cols(); i++) {
        outfile << Pcl(0, i) << " " << Pcl(1, i) << " " << Pcl(2, i) << " "
                << Pcl_C(0, i) << " " << Pcl_C(1, i) << " " << Pcl_C(2, i) << std::endl;
    }

    outfile.close();
    std::cout << "Reconstructing 3D point... Done.\r" << std::flush;
    std::cout << std::endl;
}

// Function to calculate rotation and translation between two point clouds.
void roto_traslPC(const Matrix4d R_T, std::ofstream& outfile)
{
    double rot_X = std::atan2(R_T(2, 1), R_T(2, 2)) * 180 / 3.14;
    double rot_Y = std::atan2(R_T(0, 2), R_T(0, 0)) * 180 / 3.14;
    double rot_Z = std::atan2(R_T(1, 0), R_T(0, 0)) * 180 / 3.14;

    std::cout << " Rot ERROR " << std::endl;
    std::cout << "X: " << rot_X << " Y: " << rot_Y << " Z: " << rot_Z << std::endl;
    std::cout << " Trasl ERROR " << std::endl;
    std::cout << "X: " << R_T(0,3) << " Y: " << R_T(1, 3) << " Z: " << R_T(2, 3) << std::endl;

    outfile << rot_X << " ";
    outfile << rot_Y << " ";
    outfile << rot_Z << " ";
    outfile << R_T(0, 3) << " ";
    outfile << R_T(1, 3) << " ";
    outfile << R_T(2, 3) << " ";
}

// Function to calculate the sum of squared differences between two point clouds.
double errorPC(const Matrix4Xd& PC_1, const Matrix4Xd& PC_2, const Matrix4d R_T, std::ofstream& outfile)
{
    Matrix4Xd err_Mat = PC_1 - PC_2;
    err_Mat = err_Mat.cwiseProduct(err_Mat);

    double res = err_Mat.colwise().sum().cwiseSqrt().rowwise().sum()(0);

    std::cout << " SSD ERROR " << std::endl;
    std::cout << res << std::endl;
    outfile << res << std::endl;

    return res;
}

// Function to perform point cloud alignment.
Matrix4d fit_transform(const Matrix4Xd& PC_1, const Matrix4Xd& PC_2, const int& centered)
{
    // Calculate the centers of the point clouds
    Vector4d center1 = PC_1.rowwise().sum() / PC_1.cols();
    Vector4d center2 = PC_2.rowwise().sum() / PC_2.cols();
    Matrix4Xd PC_1new;
    Matrix4Xd PC_2new;

    // If not centered, translate the point clouds to have their centers at the origin
    if (centered == 0) {
        Matrix4d trasl1 = Affine3d(Translation3d(-center1(0), -center1(1), -center1(2))).matrix();
        Matrix4d trasl2 = Affine3d(Translation3d(-center2(0), -center2(1), -center2(2))).matrix();

        PC_1new = trasl1 * PC_1;
        PC_2new = trasl2 * PC_2;
    } else {
        PC_1new = PC_1;
        PC_2new = PC_2;
    }

    // Calculate the covariance matrix
    Matrix3d cov = (PC_1new.block(0,0,3, PC_1.cols()) * PC_2new.block(0, 0, 3, PC_1.cols()).transpose()) / double(PC_1new.cols() - 1);

    // Perform Singular Value Decomposition (SVD) to perfort Principal Cmponent Analysis (PCA) 
    JacobiSVD<MatrixXd> svd;
    svd.compute(cov, ComputeFullU | ComputeFullV);

    MatrixXd S = svd.singularValues();
    MatrixXd U = svd.matrixU();
    MatrixXd V = svd.matrixV();

    Matrix3d R = V * U.transpose();

    Vector3d T;
    Matrix4d R_T;

    if (centered == 0) {
        T = center2.head(3) - (R * center1.head(3));
        R_T = Affine3d(Translation3d(T(0), T(1), T(2))).matrix();
    } else {
        R_T = Affine3d(Translation3d(0, 0, 0)).matrix();
    }

    R_T.block(0, 0, 3, 3) = R;

    return  R_T;
}

// Template function to create a KD-tree for point cloud matching.
template <typename num_t>
MatrixXd kdtree(const Matrix4Xd& mat1, const Matrix4Xd& mat2)
{
    int rows = mat1.rows();
    int cols = mat1.cols();

    // Define the KD-tree data structure
    typedef nanoflann::KDTreeEigenMatrixAdaptor<
        Eigen::Matrix<num_t, Dynamic, Dynamic>, Dynamic, nanoflann::metric_L2>
        kd_tree_t;

    MatrixXd idx(cols, 2);

    MatrixXd mat_ref = mat2.block(0, 0, 3, mat2.cols()).transpose();

    // Build the KD-tree
    kd_tree_t mat_index(3, std::cref(mat_ref), 10 /* max leaf */);

    // std::cref -> std::reference_wrapper is a class template that wraps a reference in a copyable, 
	// assignable object. It is frequently used as a mechanism to store references inside standard 
	// containers (like std::vector) which cannot normally hold references.
    
    mat_index.index->buildIndex();

    std::vector<num_t> query_pt(rows);
    for (int i = 0; i < cols; i++) {
        // Query point:
        for (int d = 0; d < rows - 1; d++) {
            query_pt[d] = mat1(d, i);
        }

        // Perform a k-nearest neighbor search
        const size_t num_results = 2; // 2 nearest neighbors will be returned
        std::vector<size_t> ret_indexes(num_results); // indexes of neighbors
        std::vector<num_t> out_dists_sqr(num_results); // square distance of them

        nanoflann::KNNResultSet<num_t> resultSet(num_results);
        resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);

        mat_index.index->findNeighbors(resultSet, &query_pt[0],
            nanoflann::SearchParams(10));

        idx(i, 0) = ret_indexes[0];
        idx(i, 1) = out_dists_sqr[0];
    }

    return idx;
}

// Function to perform the Iterative Closest Point (ICP) algorithm for point cloud registration.
MatrixCollection tricp(const MatrixCollection& PC_1, const MatrixCollection& PC_2, const MatrixCollection& PC_ground, const int max_iter, std::ofstream& outfile, const std::string& output_file)
{
    double prev_error = 0.0;
    double NPo = 0;
    int count;

    MatrixCollection tot_matrix;

    Matrix4Xd target_1(4, PC_1.mat.cols());
    Matrix4Xd target_2(4, PC_1.mat.cols());
    target_1.setZero();
    target_2.setZero();
    Matrix4Xd target_1crop;
    Matrix4Xd target_2crop;

    Matrix4Xd PC_1new = PC_1.mat;
    Matrix4d R_T;
    MatrixXd idx(PC_1.mat.cols(), 3);

    for (int iter = 0; iter < max_iter; iter++) {
        count = 0;
        std::vector<VectorXd> vec;
        idx.block(0, 1, PC_1.mat.cols(), 2) = kdtree<double>(PC_1new, PC_2.mat);

        for (int i = 0; i < PC_1.mat.cols(); i++) {
            idx(i, 0) = i;
            vec.push_back(idx.row(i));
        }
        std::sort(vec.begin(), vec.end(),
            [](VectorXd const& t1, VectorXd const& t2) { return t1(2) < t2(2); });

        NPo = 0.3 * vec.size();
        VectorXi pick = VectorXi::LinSpaced(NPo, 0, NPo);
        std::random_shuffle(pick.begin(), pick.end());

        for (int i = 0; i < int(NPo); i++) {
            if (idx(pick(i), 2) < 0.5) {
                idx.row(count) = vec[pick(i)];
                target_1.col(count) = PC_1new.col(idx(pick(i), 0));
                target_2.col(count) = PC_2.mat.col(idx(pick(i), 1));
                count++;
            }
        }

        target_1crop = target_1;
        target_2crop = target_2;
        target_1crop.conservativeResize(4, count - 1);
        target_2crop.conservativeResize(4, count - 1);

        R_T = fit_transform(target_1crop, target_2crop, 1);
        roto_traslPC(R_T, outfile);
        PC_1new = R_T * PC_1new;
        prev_error = errorPC(PC_1new, PC_ground.mat, R_T, outfile);

        std::cout << "----------" + std::to_string(iter) + "------------------" << std::endl;
    }

    tot_matrix.mat = PC_1new;
    tot_matrix.mat_C = PC_1.mat_C;

    return tot_matrix;
}

// Function to merge two point clouds using ICP.
MatrixCollection merge_icp(const MatrixCollection& PC_1, const MatrixCollection& PC_2, const std::string& output_file)
{
    int count_out = 0;

    Matrix4Xd target_3(4, PC_1.mat.cols());
    target_3.setZero();
    Matrix4Xd target_3crop;

    Matrix3Xd target_3_C(3, PC_1.mat_C.cols());
    target_3_C.setZero();
    Matrix3Xd target_3crop_C;

    MatrixXd idx(PC_1.mat.cols(), 3);

    idx.block(0, 1, PC_1.mat.cols(), 2) = kdtree<double>(PC_1.mat, PC_2.mat);
    for (int i = 0; i < PC_1.mat.cols(); i++) {
        if (idx(i, 2) > 0.001) {
            target_3.col(count_out) = PC_1.mat.col(i);
            target_3_C.col(count_out) = PC_1.mat_C.col(i);
            count_out++;
        }
    }

    target_3crop = target_3;
    target_3crop.conservativeResize(4, count_out - 1);
    target_3crop_C = target_3_C;
    target_3crop_C.conservativeResize(3, count_out - 1);

    savePC(target_3crop, target_3crop_C, output_file + "ProvaTot");

    MatrixCollection PC;
    PC.mat = target_3crop;
    PC.mat_C = target_3crop_C;
    MatrixCollection out_PC = mergePC(PC, PC_2);

    return out_PC;
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " XYZ1 XYZ2 MAX_ITER OUTPUT_FILE" << std::endl;
        return 1;
    }

    int max_iteration = atoi(argv[3]);
    const std::string output_file = argv[4];
    const std::string ply_1 = argv[1];
    const std::string ply_2 = argv[2];

    MatrixCollection PC_1 = PlytoXYZRGB(ply_1);
    MatrixCollection PC_2 = PlytoXYZRGB(ply_2);

    std::stringstream out_values;
    out_values << output_file + "_merge" << ".csv";
    std::ofstream outfile(out_values.str());

    MatrixCollection PC_1new;
    MatrixCollection final_PC;

    PC_1new = tricp(PC_1, PC_2, PC_1, max_iteration, outfile, output_file);
    final_PC = merge_icp(PC_1new, PC_2, output_file);
    savePC(final_PC.mat, final_PC.mat_C, output_file + "_merged_tricp");

    return 0;
}