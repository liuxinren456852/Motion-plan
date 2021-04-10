#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    // 地图下边界
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    // 地图上边界
    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    // 地图的栅格坐标边界
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    data = new uint8_t[GLXYZ_SIZE];
    // 内存处理
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
    
    // 分配栅格地图
    GridNodeMap = new GridNodePtr ** [GLX_SIZE];    //三级指针
    for(int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++){
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE;k++){
                Vector3i tmpIdx(i,j,k);
                Vector3d pos = gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos); //以栅格点坐标和真实坐标初始化一个栅格点
            }
        }
    }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids()
{   
    for(int i=0; i < GLX_SIZE ; i++)
        for(int j=0; j < GLY_SIZE ; j++)
            for(int k=0; k < GLZ_SIZE ; k++)
                resetGrid(GridNodeMap[i][j][k]);
}


void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      

    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1; // data用来储存对应栅格点有没有障碍物
}

vector<Vector3d> AstarPathFinder::getVisitedNodes()
{   
    vector<Vector3d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++){   
                //if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
                if(GridNodeMap[i][j][k]->id == -1)  // visualize nodes in close list only
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
{   
    neighborPtrSets.clear();
    edgeCostSets.clear();
    /*
    *
    STEP 4: finish AstarPathFinder::AstarGetSucc yourself 
    please write your code below
    *
    *
    */

    Eigen::Vector3i current_index = currentPtr->index;
    int current_x = current_index[0];
    int current_y = current_index[1];
    int current_z = current_index[2];

    int n_x,n_y,n_z;
    GridNodePtr tmp_ptr = NULL;
    Eigen::Vector3i tmp_index;

    for(int i=-1;i <=1 ;++i){
        for(int j=-1;j<=1;++j){
            for(int k = -1;k<=1;++k){

                if( i==0 && j ==0 && k==0){
                    continue;
                }
                
                n_x = current_x + i;
                n_y = current_y + j;
                n_z = current_z + k;

                if( (n_x < 0 )|| (n_y < 0) || (n_z <0) || (n_x > GLX_SIZE -1) ||  (n_y > GLY_SIZE -1) || (n_z > GLZ_SIZE -1)){
                    continue;
                }

                if(isOccupied(n_x,n_y,n_z)){
                    continue;
                }

                tmp_ptr = GridNodeMap[n_x][n_y][n_z];

                double dist = getHeu(currentPtr,tmp_ptr);

                neighborPtrSets.push_back(tmp_ptr);
                edgeCostSets.push_back(dist);


            }
        }
    }

        
}

// 启发函数计算
double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    /* 
    choose possible heuristic function you want
    Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
    Remember tie_breaker learned in lecture, add it here ?
    *
    *
    *
    STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    please write your code below
    *
    *
    */
    int distance_norm = Diagonal;
    double h ;
    Eigen::Vector3i start_index = node1->index;
    Eigen::Vector3i end_index = node2->index;

    switch (distance_norm)
    {
    case Euclidean:
        {
        double dx = abs((double)(start_index(0) - end_index(0)));
        double dy = abs((double)(start_index(1) - end_index(1)));
        double dz = abs((double)(start_index(2) - end_index(2)));
        h = std::sqrt((std::pow(dx,2.0) + std::pow(dy,2.0)+std::pow(dz,2.0)));
        break;}
    case Manhattan:
        {
        double dx = abs((double)(start_index(0) - end_index(0)));
        double dy = abs((double)(start_index(1) - end_index(1)));
        double dz = abs((double)(start_index(2) - end_index(2)));
        h = dx + dy + dz;
        break;}
    case L_infty:
        {
        double dx = abs((double)(start_index(0) - end_index(0)));
        double dy = abs((double)(start_index(1) - end_index(1)));
        double dz = abs((double)(start_index(2) - end_index(2)));
        h = std::max({dx,dy,dz});}
        break;
    case Diagonal:
        {
        double distance[3];
        distance[0] = abs((double)(start_index(0) - end_index(0)));
        distance[1] = abs((double)(start_index(1) - end_index(1)));
        distance[2] = abs((double)(start_index(2) - end_index(2)));
        std::sort(distance,distance+3);
        h = distance[0] + distance[1] + distance[2] +(std::sqrt(3.0)-3) * distance[0] + (std::sqrt(2.0)-2)*distance[1];
        break;}
    
    default:
        break;
    }


    if(use_Tie_breaker){
        double p = 1/25;
        h = h *(1.0+p);
    }

    return h;
}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{   
    // 记录路径搜索需要的时间
    ros::Time time_1 = ros::Time::now();    

    // 记录起点和终点对应的栅格坐标
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);

    // 目标索引
    goalIdx = end_idx;

    // 起点和终点的位置(应该可以省略)
    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);


    // 初始化起点和终点节点(因为前面已经初始化过，所以不需要再New)
    GridNodePtr startPtr = GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr   = GridNodeMap[end_idx(0)][end_idx(1)][end_idx(2)];

    // 待弹出点集
    openSet.clear();


    // 定义要弹出的节点
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    //计算启发函数
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr);   



    // 将起点加入开集
    // 自由点为0 闭集为-1 开集为1
    startPtr -> id = 1; 
    startPtr->nodeMapIt = openSet.insert( make_pair(startPtr -> fScore, startPtr) );

    // 预先定义拓展队列和cost队列
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    // this is the main loop
    while ( !openSet.empty()){

        // 弹出最大f的节点
        currentPtr = openSet.begin()->second;
        currentPtr->id = -1; // 标记为闭集

        // 从开集中移除
        openSet.erase(openSet.begin());
        Eigen::Vector3i current_idx = currentPtr->index;


        // 获取拓展集合
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);     


        // 遍历拓展集合        
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            

            neighborPtr = neighborPtrSets[i];
            double gh = currentPtr->gScore + edgeCostSets[i];
            double fh = gh + getHeu(neighborPtr,endPtr);

            // 如果为自由节点
            if(neighborPtr -> id == 0){ 
                
                // 计算相应的g和f，并加入opensets
                neighborPtr->gScore = gh;
                neighborPtr->fScore = fh;
                neighborPtr->cameFrom = currentPtr;

                neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore,neighborPtr));// 此处注意一定要先计算和赋值完f再加入

                // 判断是否为目标节点 改到此处为了提高代码效率 不用将所有节点加入后等弹出时发现目标再退出
                if(neighborPtr->index == goalIdx){
                    ros::Time time_2 = ros::Time::now();
                    terminatePtr = neighborPtr;
                    ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );    
                    return;
                }
                else{
                    // 标记为open list
                    neighborPtr->id = 1;
                    continue;

                }
            }
            else if(neighborPtr ->id == 1){ 
                // 如果已经在openlist里面
                if(neighborPtr->gScore > gh)
                {
                    // 更新对应的f值

                    neighborPtr->gScore = gh;
                    neighborPtr->fScore = fh;
                    neighborPtr->cameFrom = currentPtr;
                    openSet.erase(neighborPtr->nodeMapIt);
                    neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore,neighborPtr));


                }
            }
            else{
                // 如果是closelist里面的则不做处理
                continue;
            }
        }      
    }
    //if search fails
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
}


vector<Vector3d> AstarPathFinder::getPath() 
{   
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;
    /*
    *
    *
    STEP 8:  trace back from the curretnt nodePtr to get all nodes along the path
    please write your code below
    *      
    */

    GridNodePtr tmp_ptr = terminatePtr;
    while(tmp_ptr->cameFrom != NULL )
    {
        gridPath.push_back(tmp_ptr);
        tmp_ptr = tmp_ptr->cameFrom;
    }

    for (auto ptr: gridPath)
        path.push_back(ptr->coord);

        
    reverse(path.begin(),path.end());

    ROS_WARN("pathsize:%d",gridPath.size());

    return path;
}