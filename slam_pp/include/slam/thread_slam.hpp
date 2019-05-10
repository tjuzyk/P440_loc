#ifndef THREAD_SLAM_HPP
#define THREAD_SLAM_HPP

#include "slam/slam_abstract.hpp"
#include "slam/uwb_localization.hpp"


namespace uavos{

class SLAM_Controller{
public:
    explicit SLAM_Controller():m_slam_thread(), m_nh("~slam"){
        m_stop_thread = false;

        m_p_slam_callback_queue = boost::shared_ptr<ros::CallbackQueue>(new ros::CallbackQueue() );
        m_nh.setCallbackQueue(m_p_slam_callback_queue.get());    //boost::shared_ptr的成员函数get()返回一个原始的指针
             //设置回调队列   等于 m_nh.setCallbackQueue(&ros::CallbackQueue的对象)
        
    }    
    ~SLAM_Controller(){
        m_stop_thread = true;
        if(m_slam_thread.joinable()){
            m_slam_thread.join();
        }
    }

    void start();

private:
    ros::NodeHandle m_nh;
    boost::shared_ptr<ros::CallbackQueue> m_p_slam_callback_queue;  
    //创建一个指向ros::CallbackQueue类的对象的共享指针

    
    bool m_stop_thread;
    boost::thread m_slam_thread;

    void thread_slam();

};

}


#endif // THREAD_SLAM_HPP
