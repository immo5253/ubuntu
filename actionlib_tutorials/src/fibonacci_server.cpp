#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_tutorials/FibonacciAction.h>

class FibonacciAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<actionlib_tutorials::FibonacciAction> as_; // actionlib实例化变量
  std::string action_name_;
  // 创建用于发布的msg feedback/result
  actionlib_tutorials::FibonacciFeedback feedback_;
  actionlib_tutorials::FibonacciResult result_;

public:
  //构造函数，初始化创建action服务器。需要提供action名称，节点实例变量，可选的executeCB变量
  FibonacciAction(std::string name) :
    as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~FibonacciAction(void)
  {
  }
  //executeCB函数引用在构造函数中创建，回调函数传递boost的共享指针类型ConstPtr的goal作为参数
  void executeCB(const actionlib_tutorials::FibonacciGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // 确定开始的数字
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);

    // 打印开始信息
    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    // 开始执行action
    for(int i=1; i<=goal->order; i++)
    {
      // 检查优先权
      // 当客户端请求当前取消的目标优先处理，action服务器就会清除相关内容，并调用setPreempted()函数。
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      // Fibonacci数列会存放在变量feedback里，并发布这个feedback，然后进入新一轮的循环
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }
    // 当计算完成，action服务器就会通知客户端已经完成，并发送最后的结果
    if(success)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "fibonacci");
  // main函数实现开始action，并启动线程处理，运行和等待去接受goal值传入
  FibonacciAction fibonacci("fibonacci");
  ros::spin();

  return 0;
}

