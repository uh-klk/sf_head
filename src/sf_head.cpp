// - xml folder should be in the same directory as the generated exefile
//rostopic pub /sf_tune/sf_tune_filename std_msgs/String Happy.xml

#include "ros/ros.h"
#include "headMovement_parser.hpp"
#include "std_msgs/String.h"  //for receiving filename 
#include "std_msgs/Float64MultiArray.h"
#include "dynamixel_msgs/JointState.h"
#include <math.h>

class SFHeadMovement
{
public:

    SFHeadMovement(string xml_path, ros::NodeHandle nh) : xml_path_(xml_path), node_(nh)
    {
        //TODO: need create a param for changing the path

        head_movement_parser_ = new BehaviourParser(xml_path);
    }

    ~SFHeadMovement() {}

    void init();

    void receiveCB(const std_msgs::String::ConstPtr& msg);
    void receiveJointStateCB(const dynamixel_msgs::JointState::ConstPtr& msg);
    void processJoinCallbackData(int, int);
    bool isMoving(void);

    void parseBehaviour(char *xml_filename)
    {
         head_movement_parser_->requestBehaviour(xml_filename);
    }

    bool isCommandExecuted(void)
    {
        return !command_to_be_executed_;
    }

    void publishBehaviourSequence();

protected:
    ros::NodeHandle node_;
    ros::Subscriber subscriber_;

    ros::Subscriber head_pan_subscriber_;
    ros::Subscriber head_roll_subscriber_;
    ros::Subscriber neck_upper_subscriber_;
    ros::Subscriber neck_lower_subscriber_;
    ros::Subscriber tray_joint_subscriber_;

    ros::Publisher publisher_;
    ros::Publisher head_pan_publisher_;
    ros::Publisher head_roll_publisher_;
    ros::Publisher neck_upper_publisher_;
    ros::Publisher neck_lower_publisher_;
    ros::Publisher tray_joint_publisher_;

    string xml_path_;
    BehaviourParser *head_movement_parser_;
    int is_joint_moving_[4];
    bool command_to_be_executed_;
};

void SFHeadMovement::init()
{

    command_to_be_executed_ = false;

    subscriber_ = node_.subscribe("sf_headMovement_filename", 10, &SFHeadMovement::receiveCB, this);

    head_pan_subscriber_ = node_.subscribe("/head_pan_controller/state", 100, &SFHeadMovement::receiveJointStateCB, this);
    head_pan_publisher_ = node_.advertise<std_msgs::Float64MultiArray>("/head_pan_controller/command_with_speed",1000);

    head_roll_subscriber_ = node_.subscribe("/head_roll_controller/state", 100, &SFHeadMovement::receiveJointStateCB, this);
    head_roll_publisher_ = node_.advertise<std_msgs::Float64MultiArray>("/head_roll_controller/command_with_speed",1000);

    neck_upper_subscriber_ = node_.subscribe("/neck_upper_controller/state", 100, &SFHeadMovement::receiveJointStateCB, this);
    neck_upper_publisher_ = node_.advertise<std_msgs::Float64MultiArray>("/neck_upper_controller/command_with_speed",1000);

    neck_lower_subscriber_ = node_.subscribe("/neck_lower_controller/state", 100, &SFHeadMovement::receiveJointStateCB, this);
    neck_lower_publisher_ = node_.advertise<std_msgs::Float64MultiArray>("/neck_lower_controller/command_with_speed",1000);
}
void SFHeadMovement::receiveCB(const std_msgs::String::ConstPtr& msg)
{
    char *xml_filename;
    ROS_INFO("I heard : [%s]", msg->data.c_str()); //filename
    xml_filename = (char *) msg->data.c_str();
    parseBehaviour(xml_filename); //setup the behaviour

}
void SFHeadMovement::receiveJointStateCB(const dynamixel_msgs::JointState::ConstPtr& msg)
{
    int jointId = -1;
    string head_pan_joint, head_roll_joint, neck_upper_joint, neck_lower_joint;

    head_pan_joint = "head_pan_joint";
    head_roll_joint = "head_roll_joint";
    neck_upper_joint = "neck_upper_joint";
    neck_lower_joint = "neck_lower_joint";

    if (!head_pan_joint.compare(msg->name))
        jointId = 3;
    else if (!head_roll_joint.compare(msg->name))
        jointId = 2;
    else if (!neck_upper_joint.compare(msg->name))
        jointId = 1;
    else if (!neck_lower_joint.compare(msg->name))
        jointId = 0;
    else return;

    processJoinCallbackData(jointId, msg->is_moving);

}
void SFHeadMovement::processJoinCallbackData(int jointId, int jointMovingState)
{
    if (jointMovingState != is_joint_moving_[jointId])  {
        ROS_INFO("Joint %d is in moving state [%d]", jointId, jointMovingState);
        is_joint_moving_[jointId] = jointMovingState;
    }
}
bool SFHeadMovement::isMoving(void)
{
    int total = 0;

    for (int i=0; i<4; i++)
        total += is_joint_moving_[i]; //updated by callback function

    if (total == 0)
        return false;
    else {
        command_to_be_executed_ = false;  //set to false once it start executing the command
        return true;
    }
}

void SFHeadMovement::publishBehaviourSequence()
{
    char action_array[40];
    int action_array_size;

    std_msgs::Float64MultiArray action_msg_pan, action_msg_roll;
    std_msgs::Float64MultiArray action_msg_upper, action_msg_lower;

    action_msg_lower.data.clear();
    action_msg_upper.data.clear();
    action_msg_roll.data.clear();
    action_msg_pan.data.clear();

    if(head_movement_parser_->getRequestedBehaviour(action_array, &action_array_size))
    {
        ROS_INFO("Action Array Size-> %d", action_array_size);

        action_msg_lower.data.push_back(action_array[0]*M_PI/180); //joint angle in radian
        action_msg_lower.data.push_back(action_array[1]*M_PI/180);   //speed not in radian
        neck_lower_publisher_.publish(action_msg_lower);

        action_msg_upper.data.push_back(action_array[2]*M_PI/180);
        action_msg_upper.data.push_back(action_array[3]*M_PI/180);
        neck_upper_publisher_.publish(action_msg_upper);


        action_msg_roll.data.push_back(action_array[4]*M_PI/180);
        action_msg_roll.data.push_back(action_array[5]*M_PI/180);
        head_roll_publisher_.publish(action_msg_roll);

        action_msg_pan.data.push_back(action_array[6]*M_PI/180);
        action_msg_pan.data.push_back(action_array[7]*M_PI/180);
        head_pan_publisher_.publish(action_msg_pan);

        command_to_be_executed_= true; // this is to make sure every command get executed
    }
}
int main(int argc, char* argv[])
{
    string head_XML_path = "behaviourXML/head/"; //default path
    ROS_INFO("%s", head_XML_path.c_str());

    ros::init(argc, argv, "sf_head");
    ros::NodeHandle n(std::string("~"));

    n.getParam("head_XML_path",head_XML_path);    //load the param from launch file into the variable
    ROS_INFO("%s", head_XML_path.c_str());

    SFHeadMovement *sf_headMovement = new SFHeadMovement(head_XML_path, n);

    sf_headMovement->init(); //init subscriber and publisher

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        //if head is not moving and previous command has been executed
        if (!sf_headMovement->isMoving() && sf_headMovement->isCommandExecuted())
            sf_headMovement->publishBehaviourSequence();
    }

    delete sf_headMovement;

    return 0;
}
