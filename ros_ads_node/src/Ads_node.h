#ifndef HEADER_H_ADS_NODE
#define HEADER_H_ADS_NODE

//YAML include
#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "Ads_Interface.h"
#include <ros_ads_msgs/msg/ads.hpp>
#include <ros_ads_msgs/msg/state.hpp>
#include <ros_ads_msgs/ADSDecode.hpp>


//Standard includes
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <set>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

using namespace std::chrono_literals;

namespace ads_node {

class ADSNode : public rclcpp::Node
{

public:

    /**
     * @brief ADSNode::ADSNode initialize the parameters of the node and it's variables
     */
    ADSNode(rclcpp::NodeOptions options);

    /**
     * @brief ADSNode::update update internal variable values memory
     *
     * Starts by verifying and updating connection state
     *
     * @param timer_rate rate to update at
     */
    void update();

    /**
     * @brief ADSNode::publishState publish state periodically
     * @param timer_rate rate to publish at
     */
    void publishState();

    /**
     * @brief ADSNode::publishOnEvent publish variables if an event occured
     * @param timer_rate rate to verify if an event occured at
     */
    void publishOnEvent();

    /**
     * @brief ADSNode::publishOnTimer publish variables periodically
     * @param timer_rate rate to publish at
     */
    void publishOnTimer();

    /**
     * @brief ADSNode::Subscriber send the ADS device the received commands
     * @param msg the command message
     * @return true if the command was successfully sent
     */
    bool Subscriber(ros_ads_msgs::msg::Ads::SharedPtr p_msg);

private:

    bool m_publish{false};                                     /*!< Control to publish event message                          */
    bool m_configOK{false};                                    /*!< Configuration state to the modbus device                  */

    int m_rate_update;                                         /*!< update and event detection rate                           */
    int m_rate_state;                                          /*!< state publication rate                                    */
    int m_rate_publish;                                        /*!< periodical publish rate                                   */

    map<string, pair<string, double>> m_publish_on_timer;      /*!< caracteristics of variables to publish periodically       */
    map<string, pair<string, double>> m_publish_on_event;      /*!< caracteristics of variables to publish on event           */
    map<string, pair<string, double>> m_variables;             /*!< caracteristics of declared variables                      */
    mutex m_publish_on_timer_guard;                            /*!< m_publish_on_timer map guard                              */
    mutex m_publish_on_event_guard;                            /*!< m_publish_on_event map guard                              */
    mutex m_variables_guard;                                   /*!< m_variables map guard                                     */

    pair<string, double> m_checker_temp_value;                 /*!< temporary value for event detection                       */

    RosAds_Interface m_ADS = RosAds_Interface();               /*!< ADS device interface                                      */
    //ROS parameters
    int m_sub_queue_size = declare_parameter<int>("sub_queue_size",10);                                                              /*!< Queue size for subscribers                  */
    int m_pub_queue_size = declare_parameter<int>("pub_queue_size", 5);                                                              /*!< Queue size for publishers                   */
    std::string m_name = declare_parameter<std::string>("name", "test_device");                                                      /*!< Device name                                 */
    std::string m_config_file = declare_parameter<std::string>("config_file", "FULL/PATH/TO/YOUR/config_file.yaml");       /*!< YAML device desription file full path       */                                     /*!< configuration file with the description of the ADS device */

    ros_ads_msgs::msg::Ads m_on_event_msg;                          /*!< message to publish on event                               */
    ros_ads_msgs::msg::Ads m_on_timer_msg;                          /*!< message to publish on timer                               */
    ros_ads_msgs::msg::State m_state_msg;                           /*!< state message                                             */

    rclcpp::Publisher<ros_ads_msgs::msg::State>::SharedPtr mp_state_publisher = this->create_publisher<ros_ads_msgs::msg::State>("state", rclcpp::QoS(m_pub_queue_size));                          /*!< state publisher                                           */
    rclcpp::Publisher<ros_ads_msgs::msg::Ads>::SharedPtr mp_on_event_publisher = this->create_publisher<ros_ads_msgs::msg::Ads>("report_event", rclcpp::QoS(m_pub_queue_size));                       /*!< on event publisher                                        */
    rclcpp::Publisher<ros_ads_msgs::msg::Ads>::SharedPtr mp_on_timer_publisher = this->create_publisher<ros_ads_msgs::msg::Ads>("report_timer", rclcpp::QoS(m_pub_queue_size));                       /*!< periodical publisher                                      */
    rclcpp::Subscription<ros_ads_msgs::msg::Ads>::SharedPtr mp_subscriber;                              /*!< command subscriber                                        */

    rclcpp::SubscriptionOptions m_sub_option;

    rclcpp::CallbackGroup::SharedPtr mp_callback_group_update = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::CallbackGroup::SharedPtr mp_callback_group_publisher = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);           /*!< Publisher callback group     */
    rclcpp::CallbackGroup::SharedPtr mp_callback_group_checker = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::CallbackGroup::SharedPtr mp_callback_group_timer = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::CallbackGroup::SharedPtr mp_callback_group_state = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::TimerBase::SharedPtr mp_publisher_timer;           /*!< On timer publisher timer */
    rclcpp::TimerBase::SharedPtr mp_checker_timer;             /*!< Checker timer            */
    rclcpp::TimerBase::SharedPtr mp_update_timer;              /*!< IO value update timer    */
    rclcpp::TimerBase::SharedPtr mp_state_timer;               /*!< State publisher timer    */
};

}

#endif
