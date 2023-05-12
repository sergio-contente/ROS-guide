#include <ros/ros.h>
#include <std_msgs/Int32.h>

void counterCallback(const std_msgs::Int32::ConstPtr& msg) // Define uma função de callback  que recebe um parâmetro chamado "msg" 
{
  ROS_INFO("%d", msg->data); // Printa o valor do atributo "data" do parâmetro "msg".
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "topic_subscriber"); // Inicia o Node chamado "topic_subscriber" 
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("counter", 1000, counterCallback); // Cria um objeto Subscriber que vai escutar ao tópico /counter e chamar o callback "counterCallback" a cada loop.    
    ros::spin(); // Cria o loop

    return 0;
}
