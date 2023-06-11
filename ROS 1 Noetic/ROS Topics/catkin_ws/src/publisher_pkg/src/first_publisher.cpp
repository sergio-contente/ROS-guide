#include <ros/ros.h>
#include <std_msgs/Int32.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "topic_publisher");
    ros::NodeHandle nh;
    
    ros::Publisher pub = nh.advertise<std_msgs::Int32>("counter", 1000); //Instanciação de um publisher chamado "pub" que ira publicar mensagens do tipo std_msgs::Int32 no tópico "/counter"
    ros::Rate loop_rate(2);
    
    std_msgs::Int32 count; //Intanciação de uma variável "count" do tipo "std_mgs::Int32". Ela é comumente chamada de mensagem também.
    count.data = 0; //Atribuição do valor inicial 0 ao atributo data da mensagem.
    
    while (ros::ok())
    {
        pub.publish(count); //Publicação da mensagem "count" no tópico onde o pub foi configurado para publicar (tópico "counter")
        ros::spinOnce();
        loop_rate.sleep();
        ++count.data; //Incrementamos em 1 o valor da mensagem
    }
    
    return 0;
}
