#include "trajectory_generator_waypoint.h"
#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;    
using namespace Eigen;

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint(){}
TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint(){}

// 计算多项式的阶乘
int TrajectoryGeneratorWaypoint::Factorial(int x)
{
    int fac = 1;
    for(int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}



// 获取hession矩阵，也就是矩阵Q
void TrajectoryGeneratorWaypoint::GetHession(const int n_seg, 
                const int d_order,
                const Eigen::VectorXd &Time,
                Eigen::SparseMatrix<double> & hession){
    
    int p_order = 2 * d_order - 1;
    int p_num1d = p_order + 1;
    hession.resize(n_seg * p_num1d,n_seg * p_num1d);

    hession.setZero();

    for(int k = 0; k < n_seg; ++k){

        for(int i = d_order; i < p_num1d; ++i){
            for(int j = d_order; j < p_num1d ; ++j){
                double value = 1.0*Factorial(i)/Factorial(i-d_order)*Factorial(j)
                /Factorial(j-d_order)/(i+j-2*d_order+1)*pow(Time(k),i+j-2*d_order+1);
                hession.insert(k*p_num1d + i,k*p_num1d + j) = value;
            }
        }
        //hession.block( k*p_num1d,k*p_num1d,p_num1d,p_num1d) = Q_k; 稀疏矩阵不支持
    }


}

// 在线性约束矩阵的指定位置插入系数
void TrajectoryGeneratorWaypoint::InsertCoff(const int row, 
                const int col, 
                Eigen::SparseMatrix<double> & linearMatrix , 
                const double t , 
                const int d_order,
                bool one_line ,
                bool reverse){
    
    int p_num1d = 2*d_order;

    int flag = d_order ;
    if(one_line){
        flag = 1;
    }

    Eigen::MatrixXd coff(d_order,p_num1d);

    if(d_order == 4){
        coff << 1.0, 1.0*t,1.0*pow(t,2),1.0*pow(t,3),1.0*pow(t,4),1.0*pow(t,5),1.0*pow(t,6),1.0*pow(t,7),
                0.0, 1.0, 2.0*t, 3.0*pow(t,2), 4.0*pow(t,3), 5.0*pow(t,4), 6.0*pow(t,5), 7.0*pow(t,6),
                0.0, 0.0, 2.0, 6.0*t, 12.0*pow(t,2), 20.0*pow(t,3), 30.0*pow(t,4), 42.0*pow(t,5),
                0.0, 0.0, 0.0, 6.0, 24.0*t, 60.0*pow(t,2), 120.0*pow(t,3),210.0*pow(t,4);
    }
    else if(d_order == 3){
        coff << 1.0, 1.0*t,1.0*pow(t,2),1.0*pow(t,3),1.0*pow(t,4),1.0*pow(t,5),
                0.0, 1.0, 2.0*t, 3.0*pow(t,2), 4.0*pow(t,3), 5.0*pow(t,4),
                0.0, 0.0, 2.0, 6.0*t, 12.0*pow(t,2), 20.0*pow(t,3);
    }else{
        cout << "暂时只支持minisnap和minijerk";
    }

    if(reverse){
        coff = coff *(-1.0);
    }

    for (int i = 0; i < d_order && i < flag; ++i){
        for(int j = 0; j < p_num1d; ++j){
            linearMatrix.insert(row + i,col + j) = coff(i,j);
        }
    }

}

// 获取等式约束矩阵，也就是矩阵Aeq
void TrajectoryGeneratorWaypoint::GetLinearConstraintsMatrix(const int n_seg,
                const int d_order,
                const Eigen::VectorXd &Time,
                Eigen::SparseMatrix<double> & linearMatrix){

    int p_order = 2 * d_order - 1;
    int p_num1d = p_order + 1;

    cout << "p_num1d:"<< p_num1d << endl;;
    cout << "n_seg:"<< n_seg << endl;
    linearMatrix.resize(2*d_order + (n_seg-1)*( d_order + 1),p_num1d * n_seg);

    // 起点和终点限制约束
    int row = 0;
    int col = 0;
    InsertCoff(row,col,linearMatrix,0,d_order,false,false);
    cout << "row:"<< row << endl;

    row += d_order;
    col = (n_seg - 1) * p_num1d;
    cout << "row:"<< row << endl;
    cout << "col:" << col << endl;
    InsertCoff(row,col,linearMatrix,Time(n_seg-1),d_order,false,false);


    // 中间节点的位置约束
    row += d_order;
    for(int k = 0; k < n_seg -1; ++k){
        InsertCoff(row + k , k*p_num1d ,linearMatrix,Time(k),d_order,true,false);
    }
    cout << "row:"<< row << endl;
    // 连续性约束
    row += n_seg - 1;
    for(int k = 0; k < n_seg - 1; ++k){
        InsertCoff(row, k*p_num1d ,linearMatrix,Time(k),d_order,false,false);
        InsertCoff(row, (k + 1)*p_num1d ,linearMatrix,0,d_order,false,true);
        row += d_order;
    }
    cout << "row:"<< row << endl;

}


/*

    STEP 2: Learn the "Closed-form solution to minimum snap" in L5, then finish this PolyQPGeneration function

    variable declaration: input       const int d_order,                    // the order of derivative
                                      const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
                                      const Eigen::MatrixXd &Vel,           // boundary velocity
                                      const Eigen::MatrixXd &Acc,           // boundary acceleration
                                      const Eigen::VectorXd &Time)          // time allocation in each segment
                          output      MatrixXd PolyCoeff(m, 3 * p_num1d);   // position(x,y,z), so we need (3 * p_num1d) coefficients

*/

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
            const int d_order,                    // the order of derivative
            const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
            const Eigen::MatrixXd &Vel,           // boundary velocity
            const Eigen::MatrixXd &Acc,           // boundary acceleration
            const Eigen::VectorXd &Time)          // time allocation in each segment
{
    // enforce initial and final velocity and accleration, for higher order derivatives, just assume them be 0;
    int p_order   = 2 * d_order - 1;              // the order of polynomial
    int p_num1d   = p_order + 1;                  // the number of variables in each segment

    int m = Time.size();                          // the number of segments


    MatrixXd PolyCoeff = MatrixXd::Zero(m, 3 * p_num1d);           // position(x,y,z), so we need (3 * p_num1d) coefficients

    VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m);

    /*   Produce Mapping Matrix A to the entire trajectory, A is a mapping matrix that maps polynomial coefficients to derivatives.   */
    
    // 计算Q
    MatrixXd Q = MatrixXd::Zero(p_num1d * m, p_num1d*m);
    for (int k=0; k < m; ++k){
        MatrixXd Q_k = MatrixXd::Zero(p_num1d,p_num1d);
        for(int i = d_order; i < p_num1d; ++i){
            for(int j = d_order; j < p_num1d; ++j){
                Q_k(i,j) = 1.0*Factorial(i)/Factorial(i-d_order)*Factorial(j)/Factorial(j-d_order)/(i+j-2*d_order+1)*pow(Time(k),i+j-2*d_order+1);
            }
        }
        Q.block(k*p_num1d,k*p_num1d,p_num1d,p_num1d) = Q_k;
    }

    // 计算M
    MatrixXd M = MatrixXd::Zero(m * p_num1d, m * p_num1d);
    MatrixXd coeff(d_order, p_num1d);
    coeff << 1,  1,  1,  1,  1,  1,  1,  1,
                0,  1,  2,  3,  4,  5,  6,  7,
                0,  0,  2,  6,  12, 20, 30, 42,
                0,  0,  0,  6,  24, 60, 120,210;

    
    for(int k = 0; k < m; k++){
        
        MatrixXd M_k = MatrixXd::Zero(p_num1d, p_num1d); 

        double t = Time(k);
        for(int i = 0; i < d_order; i++){
            M_k(i, i) = coeff(i, i);
        }

        for(int i = 0; i < d_order; i++){
            for(int j = i; j < p_num1d; j++){
                if( i == j){
                    M_k(i+d_order, j) = coeff(i, j) ;
                }
                else{
                    M_k(i+d_order, j) = coeff(i, j) * pow(t, j - i);
                }
            }
        }

        M.block(k*p_num1d, k*p_num1d, p_num1d, p_num1d) = M_k;
    }

    
    // 计算C_t
    int ct_rows = d_order*2*m;
    int ct_cols = d_order*2*m - (m-1)*d_order;
    MatrixXd Ct = MatrixXd::Zero(ct_rows, ct_cols);
    // 构建哈希表
    vector<int> d_vector;
    for(int k = 0; k < m; k ++){
        for(int t = 0; t < 2; t++){
            for(int d = 0; d < d_order; d++){
                d_vector.push_back(k*100+t*10+d);
            }
        }
    }
    int val, row;
    int col = 0; 

    // 固定起点状态
    int k = 0;
    int t = 0;
    int d = 0;
    for(d = 0; d < d_order; d++){
        val = k * 100 + t * 10 + d;
        auto it = std::find(d_vector.begin(), d_vector.end(), val);
        row = std::distance(d_vector.begin(), it);
        Ct(row, col) = 1;
        col += 1;
    }
    // 固定中间节点位置
    t = 1;
    d = 0;
    for(k = 0; k < m - 1; k++){
        val = k * 100 + t * 10 + d;
        auto it = std::find(d_vector.begin(), d_vector.end(), val);
        row = std::distance(d_vector.begin(), it);
        Ct(row, col) = 1;

        val = (k + 1) * 100 + (t - 1) * 10 + d;
        it= std::find(d_vector.begin(), d_vector.end(), val);
        row = std::distance(d_vector.begin(), it);
        Ct(row, col) = 1;

        col += 1;
    }

    // 固定终点状态
    k = m - 1;
    t = 1;
    d = 0;
    for(d = 0; d < d_order; d++){
        val = k * 100 + t * 10 + d;
        auto it = std::find(d_vector.begin(), d_vector.end(), val);
        row = std::distance(d_vector.begin(), it);
        Ct(row, col) = 1;
        col += 1;
    }

    // 保证连续性约束
    k = 0;
    t = 1;
    d = 1;
    for(k = 0; k < m - 1; k++){
        for(d = 1; d < d_order; d++){
            val = k * 100 + t * 10 + d;
            auto it = std::find(d_vector.begin(), d_vector.end(), val);
            row = std::distance(d_vector.begin(), it);
            Ct(row, col) = 1;

            val = (k + 1) * 100 + (t - 1) * 10 + d;
            it = std::find(d_vector.begin(), d_vector.end(), val);
            row = std::distance(d_vector.begin(), it);
            Ct(row, col) = 1;

            col += 1;
        }

    }

    MatrixXd C = Ct.transpose();
    MatrixXd M_inv = M.inverse();
    MatrixXd M_inv_t = M_inv.transpose();
    MatrixXd R = C*M_inv_t*Q*M_inv*Ct;



    // 分别对三个轴计算相应的系数
    
    int num_d_F = 2 * d_order + m - 1;
    int num_d_P = (m - 1) * (d_order - 1);

    MatrixXd R_pp = R.bottomRightCorner(num_d_P, num_d_P);


    MatrixXd R_fp = R.topRightCorner(num_d_F, num_d_P);


    for(int dim = 0; dim < 3; dim++){

        VectorXd wayPoints = Path.col(dim);
        VectorXd d_F = VectorXd::Zero(num_d_F);

        // 固定起点状态
        d_F(0) = wayPoints(0); 

        // 固定中间节点位置
        for(int i = 0; i < m - 1; i++ ){
            d_F(d_order + i) = wayPoints(i + 1);
        }
        
        // 固定终点状态
        d_F(d_order + m - 1) = wayPoints(m);


        VectorXd d_P = -1.0 * R_pp.inverse() * R_fp.transpose() * d_F;

        VectorXd d_total(d_F.rows() + d_P.rows());

        d_total << d_F, d_P;

        VectorXd poly_coef_1d = M.inverse() * Ct * d_total;

        MatrixXd poly_coef_1d_t = poly_coef_1d.transpose();


        for(int k = 0; k < m; k++){
            PolyCoeff.block(k, dim*p_num1d, 1, p_num1d) = poly_coef_1d_t.block(0,k*p_num1d, 1, p_num1d);
        }
    }

    return PolyCoeff;
}


/*

    STEP 1: Learn the numerical solution to minimum snap" in L5, then finish this PolyQPGeneration function

    variable declaration: input       const int d_order,                    // the order of derivative
                                      const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
                                      const Eigen::MatrixXd &Vel,           // boundary velocity
                                      const Eigen::MatrixXd &Acc,           // boundary acceleration
                                      const Eigen::VectorXd &Time)          // time allocation in each segment
                                      OsqpEigen::Solver &slover             // osqp slover
                          output      MatrixXd PolyCoeff(m, 3 * p_num1d);   // position(x,y,z), so we need (3 * p_num1d) coefficients

*/


Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
            const int d_order,                    // the order of derivative
            const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
            const Eigen::MatrixXd &Vel,           // boundary velocity
            const Eigen::MatrixXd &Acc,           // boundary acceleration
            const Eigen::VectorXd &Time,
            OsqpEigen::Solver &slover
            )          // time allocation in each segment
{
    // enforce initial and final velocity and accleration, for higher order derivatives, just assume them be 0;
    int p_order   = 2 * d_order - 1;                                // the order of polynomial
    int p_num1d   = p_order + 1;                                    // the number of variables in each segment

    int m = Time.size();                                            // the number of segments


    MatrixXd PolyCoeff = MatrixXd::Zero(m, 3 * p_num1d);            // position(x,y,z), so we need (3 * p_num1d) coefficients

    VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m);


    //slover.settings()->setWarmStart(true);


    // 设置变量个数
    slover.data()->setNumberOfVariables( m * p_num1d );

    //设置约束个数
    slover.data()->setNumberOfConstraints(d_order*2 + (m - 1)*(d_order+1) );

    
    // 设置H矩阵
    Eigen::SparseMatrix<double> hession;
    GetHession(m,d_order,Time,hession);
    if(!slover.data()->setHessianMatrix(hession)){
        cout << "设置hession矩阵失败";
        return Eigen::MatrixXd::Zero(1,1);
    }
    else{
        cout << "hession矩阵设置成功" << endl;
    }

    //设置线性约束矩阵
    Eigen::SparseMatrix<double> linearMatrix;
    GetLinearConstraintsMatrix(m,d_order,Time,linearMatrix);

    if(!slover.data()->setLinearConstraintsMatrix(linearMatrix)){
        cout << "设置Linear矩阵失败";
        return Eigen::MatrixXd::Zero(1,1);
    }
    else{
        cout << "Linear矩阵设置成功" << endl;
    };



    Eigen::VectorXd gradient(p_num1d * m);
    gradient.setZero();

    // 设置梯度约束
    if(!slover.data()->setGradient(gradient)){
        cout << "梯度设置失败" <<endl;
    }else{
        cout << "梯度设置成功" << endl;
    }
    


    // 设置边界，求解问题
    Eigen::VectorXd lowbound = VectorXd::Zero(d_order*2 + (m - 1)*(d_order+1));
    Eigen::VectorXd upbound = VectorXd::Zero(d_order*2 + (m - 1)*(d_order+1));

    slover.data()->setLowerBound(lowbound);
    slover.data()->setUpperBound(upbound);

    //初始化求解器
    if(!slover.isInitialized()){
        slover.initSolver();
    }


    for(int dim = 0; dim < 3; dim++){

        VectorXd wayPoints = Path.col(dim);
        

        // 起点位置
        lowbound(0) = wayPoints(0);
        upbound(0) = wayPoints(0);

        // 终点位置
        lowbound(d_order) = wayPoints(m);
        upbound(d_order) = wayPoints(m);

        // 固定中间节点位置
        for(int i = 0; i < m - 1; i++ ){
            lowbound(2*d_order + i) = wayPoints(i + 1);
            upbound(2*d_order + i) = wayPoints(i + 1);
        }
        
        // 更新边界
        slover.updateBounds(lowbound,upbound);

        // 求解
        slover.solve();

        Eigen::VectorXd poly_coef_1d = slover.getSolution();


        MatrixXd poly_coef_1d_t = poly_coef_1d.transpose();


        for(int k = 0; k < m; k++){
            PolyCoeff.block(k, dim*p_num1d, 1, p_num1d) = poly_coef_1d_t.block(0,k*p_num1d, 1, p_num1d);
        }


    }

    // 每次调用之后需要清理变量
    slover.data()->clearHessianMatrix();
    slover.data()->clearLinearConstraintsMatrix();
    slover.clearSolverVariables();
    slover.clearSolver();

    return PolyCoeff;
}