#include <ros/ros.h>
#include <string>
#include <subscriber_pkg/Pessoa.h>

void counterCallback(const subscriber_pkg::Pessoa::ConstPtr& msg) // Define uma função de callback  que recebe um parâmetro chamado "msg" 
{
  ROS_INFO("Oi, meu nome eh %s, tenho %d anos e possuo %f metros", msg->Nome.c_str(), msg->Idade, msg->Altura); // Printa o valor do atributo "data" do parâmetro "msg".
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "topic_subscriber"); // Inicia o Node chamado "topic_subscriber" 
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("pessoa", 1000, counterCallback); // Cria um objeto Subscriber que vai escutar ao tópico /counter e chamar o callback "counterCallback" a cada loop.    
    ros::spin(); // Cria o loop
    
    return 0;
}
