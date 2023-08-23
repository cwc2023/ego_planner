#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <queue>

constexpr double inf = 1 >> 20; //初始化无穷大定义
struct GridNode; //结构体前向生命 一个节点表示一个栅格点
typedef GridNode *GridNodePtr; //定义一个指向GridNode的指针


//定义GridNode 结构体  包含索引坐标  状态  gScore fScore  cameFrom
struct GridNode 
{
	enum enum_state //定义节点状态的枚举类型
	{
		OPENSET = 1,  // 在待处理集合（开放列表）中
		CLOSEDSET = 2,// 在已处理集合（关闭列表）中
		UNDEFINED = 3 //未定义
	};

	int rounds{0}; //记录调用次数 用于区分每次搜索
	enum enum_state state
	{
		UNDEFINED
	}; //节点状态 默认为未定义
	Eigen::Vector3i index; //节点栅格地图中的位置索引坐标

	double gScore{inf}, fScore{inf};  
	/*gScore 是起点到该节点的距离  fScore 是代价函数值  gScore  + 节点n到终点的距离  f(n) = g(n) + h(n) */
	GridNodePtr cameFrom{NULL}; //节点的父节点 用于找到终点后反向生成路径
};


//定义节点比较器  用于优先级队列中的节点排序
class NodeComparator
{
public:
    //重载()运算符 定义比较规则 按照fScore从小到大排序
	bool operator()(GridNodePtr node1, GridNodePtr node2)
	{
		return node1->fScore > node2->fScore; //从堆底到堆顶 降序排列  也就是小的优先级高 优先级队列
	}
};


/*------------------------------ A* 路径搜索算法类----------------------------------------------------------*/
class AStar
{
private:
	GridMap::Ptr grid_map_; //存储地图信息的智能指针 指向GridMap类

	
	// coord2gridIndexFast()是一个内联函数，用于将三维世界坐标（x,y,z）转换为格点索引（id_x,id_y,id_z）。
	inline void coord2gridIndexFast(const double x, const double y, const double z, int &id_x, int &id_y, int &id_z);

	double getDiagHeu(GridNodePtr node1, GridNodePtr node2); //对角线启发式函数 ego_planer use it 
	double getManhHeu(GridNodePtr node1, GridNodePtr node2); //曼哈顿启发式函数
	double getEuclHeu(GridNodePtr node1, GridNodePtr node2); //欧几里得启发式函数
	inline double getHeu(GridNodePtr node1, GridNodePtr node2); // getHeu()函数将在A*搜索中使用，它默认使用对角线启发式函数。

	bool ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);
	//会将起始点和终止点的欧式坐标转换为网格地图的索引，并对它们进行调整，以确保它们位于地图范围内。


	inline Eigen::Vector3d Index2Coord(const Eigen::Vector3i &index) const; //将gridmap索引坐标转换为欧式坐标
	inline bool Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const; //将欧式坐标转换为栅格（gridmap）索引坐标

	//bool (*checkOccupancyPtr)( const Eigen::Vector3d &pos );

	inline bool checkOccupancy(const Eigen::Vector3d &pos) { return (bool)grid_map_->getInflateOccupancy(pos); }
	// checkOccupancy()函数用于检查指定位置是否为障碍物或被占据。

	std::vector<GridNodePtr> retrievePath(GridNodePtr current);//在找到路径后，用于从终点回溯到起点，以获取完整的路径。

	double step_size_, inv_step_size_; //格点的大小（步长）   步长的倒数
	Eigen::Vector3d center_; //地图中心坐标 
	Eigen::Vector3i CENTER_IDX_, POOL_SIZE_; //中心点的索引  地图的大小
	const double tie_breaker_ = 1.0 + 1.0 / 10000; // 用于打破评分相同节点的平局的系数

	std::vector<GridNodePtr> gridPath_; //gridmap的路径

	GridNodePtr ***GridNodeMap_; //三维指针 指向一个指针数组 指针数组的每一个元素又是一个指针 指向一个指针数组 指针数组的每一个元素又是一个指针 指向一个GridNode
	std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet_;// 存储待处理的格点（已按代价排序）

	int rounds_{0}; //记录搜索的轮数

public:
	typedef std::shared_ptr<AStar> Ptr;//定义一个指向AStar类的智能指针

	AStar(){}; //默认构造函数
	~AStar();//析构函数

	void initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size); // 初始化函数，用于设置地图和地图参数。

	bool AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
	// 执行A*搜索的函数，输入参数是步长，起点和终点的坐标，返回值表示是否找到了路径

	std::vector<Eigen::Vector3d> getPath(); //用于获取找到的路径（以欧式坐标表示）的函数。
};

// 该函数根据启发式函数（这里使用的是对角线启发式函数）和一个打破平局的系数来计算启发式成本（heuristic cost）。
// 启发式成本是A*搜索算法中的一个重要组成部分，它估计了从当前节点到目标节点的代价。
inline double AStar::getHeu(GridNodePtr node1, GridNodePtr node2)
{
	return tie_breaker_ * getDiagHeu(node1, node2);
}
// 该函数将格点索引（整数）转换为对应的欧式坐标（浮点数）。
// 转换的公式是：(index - CENTER_IDX_) * step_size_ + center_
// 其中，CENTER_IDX_是地图中心点的索引，step_size_是格点的大小，center_是地图的中心点坐标。
inline Eigen::Vector3d AStar::Index2Coord(const Eigen::Vector3i &index) const
{
	return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
};


// 该函数将欧式坐标（浮点数）转换为对应的格点索引（整数）。
// 转换的公式是：(pt - center_) * inv_step_size_ + (0.5, 0.5, 0.5) + CENTER_IDX_
// 其中，center_是地图的中心点坐标，inv_step_size_是格点大小的倒数，CENTER_IDX_是地图中心点的索引。
// 转换后，还要检查索引是否在地图范围内（即是否在[0, POOL_SIZE_)区间内）。如果不在，就输出一个错误信息，并返回false。
inline bool AStar::Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const
{
	idx = ((pt - center_) * inv_step_size_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + CENTER_IDX_;

	if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) || idx(2) < 0 || idx(2) >= POOL_SIZE_(2))
	{
		ROS_ERROR("Ran out of pool, index=%d %d %d", idx(0), idx(1), idx(2));
		return false;
	}

	return true;
};

#endif

/*inline是C++中的一个关键字，用于向编译器提供一个建议，即将一个函数声明为内联函数。

一个内联函数的主体在编译时会被插入到每个函数调用的地方。这样可以减少函数调用的开销，因为函数调用会涉及到一系列的操作，如参数传递、栈帧设置、跳转等。当函数体很小的时候，这些开销甚至会比函数体的执行时间还要长。

然而，使用内联函数也有一些缺点。例如，因为内联函数会在每个调用处都插入其代码，如果内联函数的代码很长，或者内联函数被频繁地调用，那么它可能会导致编译出的代码体积增大。

总的来说，inline关键字通常用于优化小型函数。在本例中，inline double表示该函数是内联的，并且返回一个双精度浮点数
*/



/*A star 介绍： https://vslam.net/2021/03/20/route_planning/%E8%B7%AF%E5%BE%84%E8%A7%84%E5%88%92%EF%BC%88%E4%BA%94%EF%BC%89-A-Star%E7%AE%97%E6%B3%95/*/
