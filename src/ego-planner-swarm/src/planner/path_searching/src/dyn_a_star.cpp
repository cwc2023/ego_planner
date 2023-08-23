#include "path_searching/dyn_a_star.h"

using namespace std;
using namespace Eigen;

AStar::~AStar() //析构函数在程序结束后 自动释放内存 防止内存泄漏
{
    for (int i = 0; i < POOL_SIZE_(0); i++)
        for (int j = 0; j < POOL_SIZE_(1); j++)
            for (int k = 0; k < POOL_SIZE_(2); k++)
                delete GridNodeMap_[i][j][k];
}

void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size)
{
    POOL_SIZE_ = pool_size; //地图尺寸大小 
    CENTER_IDX_ = pool_size / 2;

    GridNodeMap_ = new GridNodePtr **[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        GridNodeMap_[i] = new GridNodePtr *[POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            GridNodeMap_[i][j] = new GridNodePtr[POOL_SIZE_(2)];
            for (int k = 0; k < POOL_SIZE_(2); k++)
            {
                GridNodeMap_[i][j][k] = new GridNode; //new GridNode 申请一个内存空间 用于存储GridNode
            }
        }
    }

    grid_map_ = occ_map;
}
/*这个函数用于计算两个节点之间的对角线的距离启发函数值，如果图形中允许斜着朝邻近的节点移动
在三维空间中，首先将对角线部分距离加入 然后对剩余的两轴进行最短路径估计，最后如果还有一轴剩余则将其添加*/
double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}
/*这个函数用于计算亮点之间的，曼哈顿距离启发函数值，如果图形中只允许朝上下左右四个方向移动，则启发函数可以使用曼哈顿距离
它的计算公式为：|x1 - x2| + |y1 - y2| + |z1 - z2|。适用于只能沿着网格边缘移动的路径规划问题。*/
double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}
/*这个函数用于计算两点之间的欧式距离启发函数值 */
double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    return (node2->index - node1->index).norm();
}

/*这个函数用于反向遍历已经找到的路径 直到回到起点*/
vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
{
    vector<GridNodePtr> path; //首先，我们定义了一个path变量，它是一个GridNodePtr指针的vector。
    path.push_back(current); //把当前的current节点加入到path中

    while (current->cameFrom != NULL)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;
}


/*/*将欧式空间坐标转换为grid_map索引值   并且修正起点和终点位置*/
bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector3d start_pt, Vector3d end_pt, Vector3i &start_idx, Vector3i &end_idx)
{ /*在Eigen库中，Eigen::Vector3d和Eigen::Vector3i都是三维向量类型，分别表示三维的双精度浮点向量和整型向量*/


        /*将欧式空间坐标转换为grid_map索引值*/
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
        return false;
    /*检查起始位置是否在障碍物中 如果在 通过归一化处理始末位置差与步长的乘积修正起点位置*/
    if (checkOccupancy(Index2Coord(start_idx)))
    {
        //ROS_WARN("Start point is insdide an obstacle.");
        do
        {
            start_pt = (start_pt - end_pt).normalized() * step_size_ + start_pt;
            if (!Coord2Index(start_pt, start_idx))
                return false;
        } while (checkOccupancy(Index2Coord(start_idx)));
    }


    /*检查终点位置  是否在障碍物中 如果在 通过归一化处理始末位置差与步长的乘积修正起点位置*/
    if (checkOccupancy(Index2Coord(end_idx)))
    {
        //ROS_WARN("End point is insdide an obstacle.");
        do
        {
            end_pt = (end_pt - start_pt).normalized() * step_size_ + end_pt;
            if (!Coord2Index(end_pt, end_idx))
                return false;
        } while (checkOccupancy(Index2Coord(end_idx)));
    }

    return true;
}

bool AStar::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt)
{
    ros::Time time_1 = ros::Time::now(); //为了判断开始搜索的时间
    ++rounds_; //一个标志位 用于判断是否搜索成功

    step_size_ = step_size; //步长 0.1 
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;

    Vector3i start_idx, end_idx; //起点和终点的索引向量 三维

    /*将欧式空间坐标转换为grid_map索引值   并且修正起点和终点位置*/
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        ROS_ERROR("Unable to handle the initial or end point, force return!");
        return false;
    }

    // if ( start_pt(0) > -1 && start_pt(0) < 0 )
    //     cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

    //初始化起点和终点的节点（指针）   GridNodePtr 在头文件中定义为 GridNode的三重指针 这里通过索引值找到对应的节点
    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];

    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
    /*这段代码定义了一个名为empty的空优先队列

        1.数据类型： GridNodePtr 即 GridNode的三重指针
        2.使用 std::vector<GridNodePtr> 作为底层的容器来存储数据
        3.使用 NodeComparator 作为比较器，用于比较两个GridNodePtr的大小

        4.std::priority_queue 是一个容器适配器，它提供了常数时间复杂度下的
                最大元素查找，对元素的插入和移动操作都是对数时间复杂度。在这个队列中，元素会被按优先级排序，队列中最优先的元素位于其顶部。
        这里，NodeComparator 中的 operator() 函数定义了优先级的比较方式。在给定的代码中，NodeComparator 的 operator() 函数是这样定义的：

    bool operator()(GridNodePtr node1, GridNodePtr node2)
      {
              return node1->fScore > node2->fScore; //从堆底到堆顶 降序排列  也就是小的优先级高 优先级队列
      }
它比较了两个节点的 fScore 属性，根据 fScore 的大小来确定节点在优先队列中的优先级。此处，fScore 值较小的节点会被视为具有更高的优先级。*/





    openSet_.swap(empty);//清空openSet_ 优先队列

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;//current 代表当前openSet_中的f(n)最小的节点

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr); //获得启发式函数值 采用对角函数 
    startPtr->state = GridNode::OPENSET; //把初始节点放入openset中
    startPtr->cameFrom = NULL;//此时父节点为空
    openSet_.push(startPtr); //把初始节点放入openset中  优先队列

    endPtr->index = end_idx;//终点节点的索引值

    double tentative_gScore; //临时的g(n)值

    int num_iter = 0;
    /*---------------------开始进入主循环---------------------------------------------------*/
    while (!openSet_.empty()) /*---------------------------------首先判断openSet_是否为空-------------------------------------------*/
    {
        num_iter++;
        current = openSet_.top();//访问最小的f(n)cost节点
        openSet_.pop(); //把访问过的节点从openSet_中删除

        // if ( num_iter < 10000 )
        //     cout << "current=" << current->index.transpose() << endl;

        /*判断是否到达终点*/
        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1) && current->index(2) == endPtr->index(2))
        {
            // ros::Time time_2 = ros::Time::now();
            // printf("\033[34mA star iter:%d, time:%.3f\033[0m\n",num_iter, (time_2 - time_1).toSec()*1000);
            // if((time_2 - time_1).toSec() > 0.1)
            //     ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
            
            //如果到达终点 返回true 和  路径
            gridPath_ = retrievePath(current);
            return true;
        }

        /*如果不是终点 我们继续  我们把当前的着一个节点放入闭集中去 */
        current->state = GridNode::CLOSEDSET; //move current node from open set to closed set.
        /*八连通搜索3*3*3 */
        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue; //搜索到自己时 终止当前循环 开启下一次的循环 也就是说不搜索自己

                    Vector3i neighborIdx;
                    neighborIdx(0) = (current->index)(0) + dx;
                    neighborIdx(1) = (current->index)(1) + dy;
                    neighborIdx(2) = (current->index)(2) + dz;
                    /*超出map大小跳出循环 */
                    if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 || neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
                    {
                        continue;
                    }
                    /*初始化 拓展邻近节点*/
                    neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
                    neighborPtr->index = neighborIdx;

                    bool flag_explored = neighborPtr->rounds == rounds_;
                    /*判断拓展的节点是否在闭集中 如果已经在了 则跳出循环*/
                    if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                    {
                        continue; //in closed set.
                    }

                    neighborPtr->rounds = rounds_;
                    /*检查拓展的邻近节点索引是否在障碍物中 如果在 则跳出循环*/
                    if (checkOccupancy(Index2Coord(neighborPtr->index)))
                    {
                        continue;
                    }

                    double static_cost = sqrt(dx * dx + dy * dy + dz * dz); //搜索过程中产生的代价
                    tentative_gScore = current->gScore + static_cost; 
                    /*如果是第一次搜索 发现新的节点（不再闭集合中 加入到开集合中 */
                    if (!flag_explored)
                    {
                        //discover a new node
                        neighborPtr->state = GridNode::OPENSET;
                        neighborPtr->cameFrom = current;//设置邻近节点的父节点为当前节点
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                        openSet_.push(neighborPtr); //put neighbor in open set and record it.
                    }
                    /*此时openset 已经加入到了拓展邻近节点 同时也在搜索新的邻近节点 更新cost 寻找最小的cost 内循环完成后继续从openset中提取最小的节点cost开始搜索*/
                    else if (tentative_gScore < neighborPtr->gScore)
                    { //in open set and need update
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);//更新完成
                    }
                }
        ros::Time time_2 = ros::Time::now();//结束搜索的时间
        if ((time_2 - time_1).toSec() > 0.2) //time_1 是开始搜索的时间 如果超过0.2s 则跳出循环
        {
            ROS_WARN("Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
            return false;
        }
    }

    ros::Time time_2 = ros::Time::now();//

    if ((time_2 - time_1).toSec() > 0.1)//如果超过0.1s 则跳出循环
        ROS_WARN("Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).toSec(), num_iter);

    return false;
}

vector<Vector3d> AStar::getPath()
{
    vector<Vector3d> path;

    for (auto ptr : gridPath_)
        path.push_back(Index2Coord(ptr->index));

    reverse(path.begin(), path.end()); //反转一下
    return path;
    //在 bspline_optimizer.cpp 处用到
}
