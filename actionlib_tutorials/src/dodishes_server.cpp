//这是actionlib的服务端
#include <actionlib_tutorials/DoDishesAction.h>
#include <actionlib/server/simple_action_server.h>

//这样定义下会用起来简洁许多
typedef actionlib::SimpleActionServer<actionlib_tutorials::DoDishesAction> Server;

class DoDishesActionServer
{
private:
        actionlib_tutorials::DoDishesFeedback feedback_;
        actionlib_tutorials::DoDishesResult result_;
public:
    DoDishesActionServer(ros::NodeHandle n):
            server(n, "do_dishes",boost::bind(&DoDishesActionServer::ExecuteCb, this, _1), false)
    {
        //注册抢占回调函数
        server.registerPreemptCallback(boost::bind(&DoDishesActionServer::preemptCb, this));
    }

    //启动服务
    void Start()
    {
        server.start();
    }

    //回调函数，当客户端发出请求时，执行此函数
    void ExecuteCb(const actionlib_tutorials::DoDishesGoalConstPtr& goal) {
        // 打印请求信息
        ROS_INFO("Received goal,the dish id is :%d", goal->dishwasher_id);

        ros::Rate rate(1);
        int cur_finished_i = 1;
        int toal_dish_num = 10;
        //反馈进度
        for(cur_finished_i = 1; cur_finished_i <= toal_dish_num; cur_finished_i++)
        {
            if(!server.isActive())break;
            ROS_INFO("Cleanning the dish::%d", cur_finished_i);
            feedback_.percent_complete = cur_finished_i/10.0;
            
            server.publishFeedback(feedback_);
            rate.sleep();
        }
        // 反馈成功信息以及结果
        
        result_.total_dishes_cleaned = cur_finished_i;
        ROS_INFO("succeed result::%d", result_.total_dishes_cleaned);
        server.setSucceeded(result_);

    }

    //中断回调函数
    void preemptCb()
    {
        if(server.isActive()){
            server.setPreempted();//强制中断
        }
    }

    Server server;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "do_dishes_server");
    ros::NodeHandle n;
    //初始化，绑定回调函数
    DoDishesActionServer actionServer(n);
    //启动服务器，等待客户端信息到来
    actionServer.Start();
    ros::spin();
    return 0;
}
