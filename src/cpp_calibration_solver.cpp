#include <cstdint>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <array>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <random>
#include <thread>
#include <string>
#include <iomanip>
#include <map>

#include "ceres/ceres.h"
#include <nlohmann/json.hpp>

#include "parallel_for.h"

double pi() { return std::atan(1)*4; }

template <typename T, size_t rows, size_t cols>
struct custom_mat
{
    std::array<T,rows * cols> values;

    custom_mat()
    {
        for (size_t index = 0; index < rows * cols; ++index)
            values[index] = {};
    }

    custom_mat(custom_mat<T,rows,cols>& other){
        for(size_t index = 0; index < rows * cols; ++index)
            values[index] = other.values[index];
    }

    custom_mat(const custom_mat<T,rows,cols>& other){
        for(size_t index = 0; index < rows * cols; ++index)
            values[index] = other.values[index];
    }

    const T &operator()(size_t row, size_t col) const
    {
        return values[row * cols + col];
    }

    T &operator()(size_t row, size_t col)
    {
        return values[row * cols + col];
    }

    void print() const
    {
        for (size_t row = 0; row < rows; ++row)
        {
            for (size_t col = 0; col < cols; ++col)
            {
                std::printf("%.4f ", values[row * cols + col]);
            }
            std::printf("\n");
        }
    }
};

template <typename T_left,typename T_right, typename T_out, size_t left_size_rows, size_t left_size_cols ,size_t right_size_rows, size_t right_size_cols>
void multiply(const custom_mat<T_left, left_size_rows, left_size_cols>& l, const custom_mat<T_right, right_size_rows, right_size_cols>& r, custom_mat<T_out, left_size_rows, right_size_cols>& result)
{
    for (size_t row = 0; row < left_size_rows; ++row)
        for (size_t col = 0; col < right_size_cols; ++col)
            for (size_t k = 0; k < left_size_cols; ++k)
                result(row, col) += l(row, k) * r(k, col);
    return;
}

constexpr size_t number_of_residuals = 12;
constexpr size_t number_of_unknowns = 12;

template <typename T>
void vec2rot(const T *transformation_parameters, custom_mat<T, 4, 4> &mat)
{
    T t1 = cos(transformation_parameters[2]);
    T t2 = sin(transformation_parameters[2]);
    T t3 = cos(transformation_parameters[1]);
    T t4 = sin(transformation_parameters[1]);
    T t5 = cos(transformation_parameters[0]);
    T t6 = sin(transformation_parameters[0]);

    T t7 = transformation_parameters[3];
    T t8 = transformation_parameters[4];
    T t9 = transformation_parameters[5];

    mat(0, 0) = t1 * t3;
    mat(1, 0) = t2 * t3;
    mat(2, 0) = -t4;
    mat(3, 0) = {};

    mat(0, 1) = t1 * t4 * t6 - t2 * t5;
    mat(1, 1) = t1 * t5 + t2 * t4 * t6;
    mat(2, 1) = t3 * t6;
    mat(3, 1) = {};

    mat(0, 2) = t2 * t6 + t1 * t4 * t5;
    mat(1, 2) = t2 * t4 * t5 - t1 * t6;
    mat(2, 2) = t3 * t5;
    mat(3, 2) = {};

    mat(0, 3) = t7;
    mat(1, 3) = t8;
    mat(2, 3) = t9;
    mat(3, 3) = {};
    mat(3, 3) += 1.0;
}

template <typename T>
void invert_homogenenous_transformation(const custom_mat<T, 4, 4> &original, custom_mat<T, 4, 4> &inverted)
{
    custom_mat<T, 3, 3> rotation;
    custom_mat<T, 3, 1> minus_original_translation;
    for (size_t row = 0; row < 3; ++row)
    {
        minus_original_translation(row, 0) = -original(row, 3);
        for (size_t col = 0; col < 3; ++col)
            rotation(row, col) = original(col, row);
    }

    custom_mat<T, 3, 1> final_inverse_translation;
    multiply(rotation, minus_original_translation, final_inverse_translation);
    for (size_t row = 0; row < 3; ++row)
    {
        inverted(row, 3) = final_inverse_translation(row, 0);
        for (size_t col = 0; col < 3; ++col)
            inverted(row, col) = rotation(row, col);
    }
    inverted(3, 3) = 1.0;
}

struct Observation
{
    custom_mat<double, 4, 4> inverse_Polaris_transformations;
    custom_mat<double, 4, 4> Polaris_transformations;
    custom_mat<double, 4, 4> Robot_transformations;
};

template <typename T>
struct ProblemInfo
{
    custom_mat<T, 4, 4> T_MarkerToFlange;
    custom_mat<T, 4, 4> T_BaseToPolaris;

    void update(const T *variables)
    {

        size_t pointer_offset = 0;
        vec2rot<T>(variables + pointer_offset, T_MarkerToFlange);
        pointer_offset += 6;
        vec2rot<T>(variables + pointer_offset, T_BaseToPolaris);
        pointer_offset += 6;
    }
};

template <typename T>
void compute_cost(const ProblemInfo<T> &local_transformation, const Observation &observation, T *residual)
{
    custom_mat<T, 4, 4> T_PolarisToFlange;
    multiply(local_transformation.T_MarkerToFlange, observation.inverse_Polaris_transformations, T_PolarisToFlange);
    custom_mat<T, 4, 4> T_PolarisToBase;
    multiply(observation.Robot_transformations, T_PolarisToFlange, T_PolarisToBase);
    custom_mat<T, 4, 4> identity_matrix;
    multiply(local_transformation.T_BaseToPolaris, T_PolarisToBase, identity_matrix);



    /* custom_mat<T, 4, 4> tempmatmat_1;
    multiply(observation.inverse_Polaris_transformations, local_transformation.T_BaseToPolaris, tempmatmat_1);
    custom_mat<T, 4, 4> tempmatmat_2;
    multiply(local_transformation.T_MarkerToFlange, tempmatmat_1, tempmatmat_2);
    custom_mat<T, 4, 4> identity_matrix;
    multiply(observation.Robot_transformations, tempmatmat_2, identity_matrix); */



    /* custom_mat<T, 4, 4> tempmatmat_1;
    multiply(local_transformation.T_BaseToPolaris, observation.Robot_transformations, tempmatmat_1);
    custom_mat<T, 4, 4> tempmatmat_2;
    multiply(observation.inverse_Polaris_transformations, tempmatmat_1, tempmatmat_2);
    custom_mat<T, 4, 4> identity_matrix;
    multiply(local_transformation.T_MarkerToFlange, tempmatmat_2, identity_matrix); */


    /* custom_mat<T, 4, 4> tempmatmat_1;
    multiply(observation.Robot_transformations, local_transformation.T_MarkerToFlange, tempmatmat_1);
    custom_mat<T, 4, 4> tempmatmat_2;
    multiply(local_transformation.T_BaseToPolaris, tempmatmat_1, tempmatmat_2);
    custom_mat<T, 4, 4> identity_matrix;
    multiply(observation.inverse_Polaris_transformations, tempmatmat_2, identity_matrix); */


    size_t linear_index = 0;
    for (size_t row = 0; row < 3; ++row){
        for (size_t col = 0; col < 4; ++col)
        {
            residual[linear_index] = (row == col) ? identity_matrix(row, col) - 1.0: identity_matrix(row, col);
            ++linear_index;
        }
    }
};

struct DualPoseResidual
{
private:
    Observation observation;

public:
    DualPoseResidual(Observation observation) : observation{observation}
    {
    }

    template <typename T>
    bool operator()(const T *const variables, T *residual) const
    {
        ProblemInfo<T> local_transformation;
        local_transformation.update(variables);
        compute_cost<T>(local_transformation, observation, residual);
        return true;
    }
};



Eigen::MatrixXd convert_matrix(std::stringstream& data, char element_separation_char, char line_separation_char)
{
    // the matrix entries are stored in this variable row-wise. For example if we have the matrix:
    // M=[a b c 
    //    d e f]
    // the entries are stored as matrixEntries=[a,b,c,d,e,f], that is the variable "matrixEntries" is a row vector
    // later on, this vector is mapped into the Eigen matrix format
    std::vector<double> matrixEntries;
 
    // this variable is used to store the row of the matrix that contains commas 
    std::string matrixRowString;
 
    // this variable is used to store the matrix entry;
    std::string matrixEntry;
 
    // this variable is used to track the number of rows
    int matrixRowNumber = 0;
 
 
    while (getline(data, matrixRowString, line_separation_char)) // here we read a row by row of matrixDataFile and store every line into the string variable matrixRowString
    {
        std::stringstream matrixRowStringStream(matrixRowString); //convert matrixRowString that is a string to a stream variable.
 
        while (getline(matrixRowStringStream, matrixEntry, element_separation_char)) // here we read pieces of the stream matrixRowStringStream until every comma, and store the resulting character into the matrixEntry
        {
            matrixEntries.push_back(stod(matrixEntry));   //here we convert the string to double and fill in the row vector storing all the matrix entries
        }
        matrixRowNumber++; //update the column numbers
    }
 
    // here we convet the vector variable into the matrix and return the resulting object, 
    // note that matrixEntries.data() is the pointer to the first memory location at which the entries of the vector matrixEntries are stored;
    return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(matrixEntries.data(), matrixRowNumber, matrixEntries.size() / matrixRowNumber);
 
}

void read_json(int& n_initializations, int& max_iterations, Eigen::MatrixXd& observations, Eigen::MatrixXd& initial_guess){
    try{
        nlohmann::json data;

        std::filesystem::path path = PROJECT_DIR;  // Example path
        std::string filename = "observations.json";  // Example filename
        std::filesystem::path filePath = path / filename;

        std::ifstream in(filePath);

        /* #ifdef _WIN32
            std::ifstream in("C:/Dev/RobotPolarisData/observations.json");
        #else
            std::ifstream in("/home/hrl/Dev/test_cal/observations.json");
        #endif */

        in >> data;

        std::string n_initializations_str = data["n_initializations"];

        std::string max_iterations_str = data["max_iterations"];

        std::string observations_str = data["observations"];

        std::string unknowns_str = data["first_initialization"];

        /* std::cout << "variables: " << unknowns_str << std::endl; */

        Eigen::MatrixXd n_initializations_matrix;
        std::stringstream matrix_strm02;
        matrix_strm02 << n_initializations_str;
        n_initializations_matrix = convert_matrix(matrix_strm02,' ', ';');
        n_initializations = n_initializations_matrix.coeff(0,0);


        Eigen::MatrixXd max_iterations_matrix;
        std::stringstream matrix_strm0;
        matrix_strm0 << max_iterations_str;
        max_iterations_matrix = convert_matrix(matrix_strm0,' ', ';');
        max_iterations = max_iterations_matrix.coeff(0,0);



        std::stringstream matrix_strm;
        matrix_strm << observations_str;
        observations = convert_matrix(matrix_strm,' ', ';');
  



        std::stringstream matrix_strm2;
        matrix_strm2 << unknowns_str;
        initial_guess = convert_matrix(matrix_strm2,' ',';');


    } catch(...){
        std::cout << "failure to read the calibration data, \nplease provide a file \"optimization_result.json\" \nwith the calibration of the set up";
        //return 0;
    }

}



int main( int argc, char *argv[] )
{

    //std::cout << "The only argument is the path for the .json file location (Ex: \"C:/Dev/Robot_OpticalSystem_Calibration/data\")\n";

    std::cout << "The project directory is: " << PROJECT_DIR << "\n" << std::endl;

    double unknowns[number_of_unknowns];
    double residual[number_of_unknowns];

    int max_iters = 5000;
    int n_initializations = 50;
    //double initial_guess[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    Eigen::MatrixXd observations;

    Eigen::MatrixXd initial_guess;


    read_json(n_initializations, max_iters, observations, initial_guess);


    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::uniform_real_distribution<> d_ang{-pi(), pi()};
    std::uniform_real_distribution<> d_dist{-3, 3};


    ceres::Problem problem;

    
    std::vector<Observation> observation;
    observation.reserve(observations.rows());

    for(size_t outer_index = 0; outer_index < observations.rows(); ++outer_index){
        int n = 0;
        Observation obser;
        for (size_t row = 0; row < 4; ++row)
        {
            for (size_t col = 0; col < 4; ++col)
            {
                
                obser.Robot_transformations(col, row) = observations(outer_index,n);
                obser.Polaris_transformations(col, row) = observations(outer_index ,n+ 16);
                ++n;
            }
        }
        invert_homogenenous_transformation(obser.Polaris_transformations, obser.inverse_Polaris_transformations);
        observation.push_back(std::move(obser));
       
    }

    for (const auto &obs : observation)
    {
        ceres::CostFunction *cost_function =
            new ceres::AutoDiffCostFunction<DualPoseResidual, number_of_residuals, number_of_unknowns>(
                new DualPoseResidual(obs));
        problem.AddResidualBlock(cost_function, nullptr, unknowns);
    }

    ceres::Solver::Options options;
    options.max_num_iterations = max_iters;

    options.linear_solver_type = ceres::DENSE_QR;
    options.check_gradients = false;
    options.minimizer_progress_to_stdout = false;

    options.function_tolerance = std::numeric_limits<double>::min();
    options.gradient_tolerance = std::numeric_limits<double>::min();
    options.inner_iteration_tolerance = std::numeric_limits<double>::min();
    options.num_threads = std::thread::hardware_concurrency();
    options.parameter_tolerance = std::numeric_limits<double>::min();

    double best_solution[number_of_unknowns];
    double best_residual = 9999999.9;

    

//////////////////////
    //parallel_for(n_initializations, [&](int start, int end){ 
    //for(int n_inits = start; n_inits < end; ++n_inits){
    for(size_t n_inits = 0; n_inits < n_initializations; ++n_inits){

        
        ceres::Problem problem;
        ceres::Solver::Summary summary;

        if(n_inits == 0){
            for(size_t n = 0; n < 12; ++n){
                unknowns[n] = initial_guess(n);
            }
        } else {
            unknowns[0] = d_ang(gen);
            unknowns[1] = d_ang(gen);
            unknowns[2] = d_ang(gen);
            unknowns[3] = d_dist(gen);
            unknowns[4] = d_dist(gen);
            unknowns[5] = d_dist(gen);
            unknowns[6] = d_ang(gen);
            unknowns[7] = d_ang(gen);
            unknowns[8] = d_ang(gen);
            unknowns[9] = d_dist(gen);
            unknowns[10] = d_dist(gen);
            unknowns[11] = d_dist(gen);
        }

        for (const auto &obs : observation)
        {
            ceres::CostFunction *cost_function =
                new ceres::AutoDiffCostFunction<DualPoseResidual, number_of_residuals, number_of_unknowns>(
                    new DualPoseResidual(obs));
            problem.AddResidualBlock(cost_function, nullptr, unknowns);
        }
        

        problem.SetParameterLowerBound(unknowns, 0, -pi());
        problem.SetParameterUpperBound(unknowns, 0, pi()); 
        problem.SetParameterLowerBound(unknowns, 1, -pi()/2);
        problem.SetParameterUpperBound(unknowns, 1, pi()/2); 
        problem.SetParameterLowerBound(unknowns, 2, -pi());
        problem.SetParameterUpperBound(unknowns, 2, pi()); 
        problem.SetParameterLowerBound(unknowns, 6, -pi());
        problem.SetParameterUpperBound(unknowns, 6, pi()); 
        problem.SetParameterLowerBound(unknowns, 7, -pi()/2);
        problem.SetParameterUpperBound(unknowns, 7, pi()/2); 
        problem.SetParameterLowerBound(unknowns, 8, -pi());
        problem.SetParameterUpperBound(unknowns, 8, pi()); 


        std::chrono::steady_clock::time_point begin1 = std::chrono::steady_clock::now();
        ceres::Solve(options, &problem, &summary);
        std::chrono::steady_clock::time_point end1 = std::chrono::steady_clock::now();
        std::cout << "Initialization number "<< n_inits+1 << "    time to solve = " << std::chrono::duration_cast<std::chrono::microseconds>(end1 - begin1).count()*1e-6 << " seconds" << std::endl;

        if(summary.final_cost < best_residual){
            size_t iter = 0;
            for (auto &val : unknowns)
            {
                best_solution[iter] = val;
                ++iter;
            }
            best_residual = summary.final_cost;
        }
    }
    //});

    std::filesystem::path path = PROJECT_DIR;  // Example path
    std::string filename = "results.txt";  // Example filename
    std::filesystem::path filePath = path / filename;

    std::ifstream in(filePath);

    std::ofstream myfile;

    myfile.open(filePath);

    double solution[12] ;
    std::cout << "\n\nSolution: \n";
    size_t iter = 0;
    for (auto &val : unknowns)
    {
        solution[iter] = best_solution[iter];
        std::cout << "  " << solution[iter];
        myfile << solution[iter] << "    ";
        ++iter;
    }

    myfile.close();
   std::cout << "\n";
}

