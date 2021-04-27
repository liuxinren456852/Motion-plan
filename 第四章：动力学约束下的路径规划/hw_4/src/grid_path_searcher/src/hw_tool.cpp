#include <hw_tool.h>

using namespace std;
using namespace Eigen;

void Homeworktool::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
}

void Homeworktool::setObs(const double coord_x, const double coord_y, const double coord_z)
{   
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      
    
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

bool Homeworktool::isObsFree(const double coord_x, const double coord_y, const double coord_z)
{
    Vector3d pt;
    Vector3i idx;
    
    pt(0) = coord_x;
    pt(1) = coord_y;
    pt(2) = coord_z;
    idx = coord2gridIndex(pt);

    int idx_x = idx(0);
    int idx_y = idx(1);
    int idx_z = idx(2);

    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

Vector3d Homeworktool::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i Homeworktool::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

Eigen::Vector3d Homeworktool::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

double Homeworktool::OptimalBVP(Eigen::Vector3d _start_position,Eigen::Vector3d _start_velocity,Eigen::Vector3d _target_position)
{
    double optimal_cost = 100000; // this just to initial the optimal_cost, you can delete it 
    /*
                    



    STEP 2: go to the hw_tool.cpp and finish the function Homeworktool::OptimalBVP
    the solving process has been given in the document

    because the final point of trajectory is the start point of OBVP, so we input the pos,vel to the OBVP

    after finish Homeworktool::OptimalBVP, the Trajctory_Cost will record the optimal cost of this trajectory


    */
    // 计算两点边界最优控制问题，此处的终止位置为固定，但速度不固定，假设终值速度为0
    // 具体公式来源参看MATLAB代码
    double pxf = _target_position(0);
    double pyf = _target_position(1);
    double pzf = _target_position(2);

    double px0 = _start_position(0);
    double py0 = _start_position(1);
    double pz0 = _start_position(2);

    double vx0 = _start_velocity(0);
    double vy0 = _start_velocity(1);
    double vz0 = _start_velocity(2);

    double vxf = 0,vyf = 0, vzf = 0;


    Eigen::Matrix<double,4,4> m;

    double c0 = -36*( (pxf-px0)*(pxf-px0) + (pyf-py0)*(pyf-py0) + (pzf-pz0)*(pzf-pz0)) ;
    double c1 = 24*((pxf-px0)*(vxf+vx0) + (pyf-py0)*(vyf+vy0) + (pzf-pz0)*(vzf+vz0));
    double c2 = -4*(vx0*vx0 + vx0*vxf + vxf*vxf + vy0*vy0 + vy0*vyf + vyf*vyf + vz0*vz0 + vz0*vzf + vzf*vzf);
    double c3 = 0.0;

    m << 0,0,0,-c0,
         1,0,0,-c1,
         0,1,0,-c2,
         0,0,1,c3;



    double J;

    Eigen::Matrix<complex<double>,Eigen::Dynamic,Eigen::Dynamic> eigenValules;
    eigenValules = m.eigenvalues();


    for(int i=0; i<4; ++i){
        double T = std::real(eigenValules(i));
        double img = std::imag(eigenValules(i));

        if(T <= 0 || std::abs(img) >= 1e-16){
            continue;
        }

        Eigen::Vector3d alpha,beta;

        alpha(0) = 12*(px0-pxf+T*vx0)/T/T/T - 6*(vx0-vxf)/T/T;
        alpha(1) = 12*(py0-pyf+T*vy0)/T/T/T - 6*(vy0-vyf)/T/T;
        alpha(2) = 12*(pz0-pzf+T*vz0)/T/T/T - 6*(vz0-vzf)/T/T;
        beta(0) = 2*(vx0-vxf)/T -6*(px0-pxf+T*vx0)/T/T;
        beta(1) = 2*(vy0-vyf)/T -6*(py0-pyf+T*vy0)/T/T;
        beta(2) = 2*(vz0-vzf)/T -6*(pz0-pzf+T*vz0)/T/T;

        J = T + 1.0/3.0*alpha.dot(alpha)*std::pow(T,3) + alpha.dot(beta)*std::pow(T,2) + beta.dot(beta)*T;

        if(J < optimal_cost){
            optimal_cost = J;
        }  


    }


    return optimal_cost;
}
