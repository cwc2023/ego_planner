
/*
用于四旋翼无人机控制的ROS（Robot Operating System）程序的主要入口点（main function）的代码。

这段代码做了以下几件事情：

1.初始化ROS，并设置该节点的名称为"px4ctrl"。
2.创建一个ROS的节点句柄（NodeHandle）实例，这个句柄可以用来执行ROS的各种操作，如发布和订阅话题。

3.定义了一个SIGINT（通常是CTRL+C）信号的处理函数mySigintHandler，当用户试图终止程序时，这个函数会被调用，使得ROS节点可以优雅地关闭。

4.从ROS参数服务器加载参数，然后实例化一个LinearControl控制器和一个PX4CtrlFSM有限状态机。

5.订阅了一系列的ROS话题，包括但不限于"mavros/state"（无人机的状态），"mavros/extended_state"（无人机的扩展状态），"odom"（无人机的位置和速度）等。这些订阅的话题数据将用于有限状态机的运行。

6.定义了一些ROS的发布器，如"fsm.ctrl_FCU_pub"，它将发布无人机的目标姿态（AttitudeTarget）到"mavros/setpoint_raw/attitude"话题。

7.定义了一些ROS的服务客户端，如"fsm.set_FCU_mode_srv"，它可以调用"mavros/set_mode"服务来改变无人机的飞行模式。

8.如果不使用遥控器（RC），则初始化一个动态参数服务器，用于接收和处理参数更新请求。

9.如果使用遥控器，程序会等待遥控器的连接。

程序会在与PX4无人机建立连接之前，持续尝试连接。

一旦建立连接，程序将进入主循环，其中每一次迭代都会运行一次状态机的process()方法。

整个程序的主要目标就是让无人机根据收到的命令（或者遥控器的指令）和无人机的当前状态，计算出无人机的目标姿态，然后发布到"mavros/setpoint_raw/attitude"话题，使无人机达到期望的状态。
*/




#include <ros/ros.h>
#include "PX4Ctrlfsm.h"
#include <signal.h>


//为中断信号（SIGINT）定义一个处理函数
void mySigintHandler(int sig)
{
    ROS_INFO("[PX4Ctrl] exit...");//输出信息
    ros::shutdown();//关闭ROS节点
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "px4ctrl");//初始化ROS节点，节点名称为"px4ctrl"
    ros::NodeHandle nh;//创建一个ROS节点句柄（NodeHandle）实例，这个句柄可以用来执行ROS的各种操作，如发布和订阅话题
    ros::NodeHandle nh_("~");//创建一个私有的ROS节点句柄（NodeHandle）实例
    //设置SIGINT信号的处理器为我们自定义的函数mySigintHandler
    signal(SIGINT, mySigintHandler);
    //程序暂停1秒
    ros::Duration(1.0).sleep();

    Parameter_t param; //创建一个参数对象
    param.config_from_ros_handle(nh_);//从ROS参数服务器初始化参数

    // Controller controller(param);
    LinearControl controller(param);//创建一个线性控制对象
    PX4CtrlFSM fsm(param, controller);//创建一个有限状态机对象

    //订阅不同的主题，并且设置回调函数
    ros::Subscriber state_sub =
        nh.subscribe<mavros_msgs::State>("mavros/state",
                                         10,
                                         boost::bind(&State_Data_t::feed, &fsm.state_data, _1));
    /*ros::Subscriber state_sub = 1. 声明了一个ros:: Subcriber的变量state_sub */
    /*   nh.subscribe<mavros_msgs::State>("mavros/state",
                                         10,
                                         boost::bind(&State_Data_t::feed, &fsm.state_data, _1));
    这里是订阅的主题函数 有三个参数
    1. mavros/state 是主题的名称， 这个主题的消息类型是 mavros_msgs::State
    2. 10 是队列的大小，如果处理的速度不够快，那么队列会被填满，后面的消息会被丢弃
    3. boost::bind(&State_Data_t::feed, &fsm.state_data, _1) 是回调函数，当有消息到达时，会调用这个函数
    这里使用了boost::bind将类fsm的成员函数 State_Data_t::feed 和对象&fsm.state_date 绑定在一起，_1是一个占位符，表示回调函数的参数
    总的来说，这段代码的作用是当 mavros/state 主题有新的mavros_msga::State 类型的消息发布时，就会
    调用fsm的成员函数State_Data_t::feed，将这个消息作为参数传递给这个函数
     */                                   

    ros::Subscriber extended_state_sub =
        nh.subscribe<mavros_msgs::ExtendedState>("mavros/extended_state",
                                                 10,
                                                 boost::bind(&ExtendedState_Data_t::feed, &fsm.extended_state_data, _1));

    ros::Subscriber odom_sub =
        nh.subscribe<nav_msgs::Odometry>("odom",
                                         100,
                                         boost::bind(&Odom_Data_t::feed, &fsm.odom_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    ros::Subscriber cmd_sub =
        nh.subscribe<quadrotor_msgs::PositionCommand>("cmd",
                                                      100,
                                                      boost::bind(&Command_Data_t::feed, &fsm.cmd_data, _1),
                                                      ros::VoidConstPtr(),
                                                      ros::TransportHints().tcpNoDelay());

    ros::Subscriber imu_sub =
        nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", // Note: do NOT change it to /mavros/imu/data_raw !!!
                                       100,
                                       boost::bind(&Imu_Data_t::feed, &fsm.imu_data, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());

    //如果没有接受到RC 则订阅RC主题
    ros::Subscriber rc_sub;
    if (!param.takeoff_land.no_RC) // mavros will still publish wrong rc messages although no RC is connected
    {
        rc_sub = nh.subscribe<mavros_msgs::RCIn>("mavros/rc/in",
                                                 10,
                                                 boost::bind(&RC_Data_t::feed, &fsm.rc_data, _1));
    }

    // ros::Subscriber bat_sub =
    //     nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery",
    //                                             100,
    //                                             boost::bind(&Battery_Data_t::feed, &fsm.bat_data, _1),
    //                                             ros::VoidConstPtr(),
    //                                             ros::TransportHints().tcpNoDelay());

    ros::Subscriber takeoff_land_sub =
        nh.subscribe<quadrotor_msgs::TakeoffLand>("takeoff_land",
                                                  100,
                                                  boost::bind(&Takeoff_Land_Data_t::feed, &fsm.takeoff_land_data, _1),
                                                  ros::VoidConstPtr(),
                                                  ros::TransportHints().tcpNoDelay());
    //向不同的主题发布消息
    fsm.ctrl_FCU_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
    fsm.traj_start_trigger_pub = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);
    //调试发布
    fsm.debug_pub = nh.advertise<quadrotor_msgs::Px4ctrlDebug>("/debugPx4ctrl", 10); // debug
    //创建服务客户端
    fsm.set_FCU_mode_srv = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    fsm.arming_client_srv = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    fsm.reboot_FCU_srv = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

    ros::Duration(0.5).sleep(); //程序暂停0.5

    //动态参数服务的设置
    dynamic_reconfigure::Server<px4ctrl::fake_rcConfig> server;
    dynamic_reconfigure::Server<px4ctrl::fake_rcConfig>::CallbackType f;

    //如果没有接受到RC 就将回调函数绑定到动态参数服务器上
    if (param.takeoff_land.no_RC)
    {
        f = boost::bind(&Dynamic_Data_t::feed, &fsm.dy_data ,_1); //绑定回调函数
        server.setCallback(f); //为服务器设置回调函数， 节点程序运行时会调用一次回调函数来输出当前的参数配置情况
        ROS_WARN("PX4CTRL] Remote controller disabled, be careful!");
    }
    else
    {
        ROS_INFO("PX4CTRL] Waiting for RC");
        //等待接受RC
        while (ros::ok())
        {
            ros::spinOnce();
            if (fsm.rc_is_received(ros::Time::now()))
            {
                ROS_INFO("[PX4CTRL] RC received.");
                break;
            }
            ros::Duration(0.1).sleep();
        }
    }
    //等待与PX4的连接
    int trials = 0;
    while (ros::ok() && !fsm.state_data.current_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (trials++ > 5)
            ROS_ERROR("Unable to connnect to PX4!!!");
    }
    //主循环
    ros::Rate r(param.ctrl_freq_max);
    while (ros::ok())
    {
        ROS_INFO_ONCE("PX4CTRL] Is OK!");
        r.sleep();
        ros::spinOnce();
        fsm.process(); // We DO NOT rely on feedback as trigger, since there is no significant performance difference through our test.
    }

    return 0;
}