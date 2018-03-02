//System includes
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

//Boost includes
#include <boost/shared_ptr.hpp>
#include <rws2018_libs/team.h>

#include <rws2018_msgs/MakeAPlay.h>

  
using namespace std;                                                                               

/**
 * @brief The namespace of this lib
 */
namespace rws_jmota
{
    class Player
    {
        public:

        //Constructor with the same name as the class
        Player(string argin_name)
        {
            name = argin_name;
        }

        int setTeamName(int team_index = 0 /*default value*/)
        {
            switch (team_index)
            {
                case 0: 
                    return setTeamName("red"); break;
                case 1: 
                    return setTeamName("green"); break;
                case 2: 
                    return setTeamName("blue");  break;
                default: 
                    ROS_WARN("wrong team index given. Cannot set team");
                    break;
            }
        }

        //Set team name, if given a correct team name (accessor)
        int setTeamName(string argin_team)
        {
            if (argin_team=="red" || argin_team=="green" || argin_team=="blue")
            {
                team = argin_team;
                return 1;
            }
            else
            {
                ROS_WARN("cannot set team name to %s",argin_team.c_str());
                // cout << "cannot set team name to " << argin_team << endl;
                return 0;
            }
        }

        //Gets team name (accessor)
        string getTeamName(void) {return team;}

        //A public atribute
        string name;

        private:
        string team;

    };


    class MyPlayer: public Player
    {
        public: 

        boost::shared_ptr<Team> red_team;
        boost::shared_ptr<Team> green_team;
        boost::shared_ptr<Team> blue_team;   

        boost::shared_ptr<Team> my_team;
        boost::shared_ptr<Team> my_preys;
        boost::shared_ptr<Team> my_hunters;   

        tf::TransformBroadcaster br; //declare the broadcaster 
        // float x=-5.0, y=5.0;  
        // float rx, ry;
        ros::NodeHandle n;
        boost::shared_ptr<ros::Subscriber> sub;

        
        MyPlayer(string argin_name, string argin_team/*disregard*/): Player(argin_name)
        {
            red_team = boost::shared_ptr<Team> (new Team("red"));

            green_team = boost::shared_ptr<Team> (new Team("green"));

            blue_team = boost::shared_ptr<Team> (new Team("blue"));

            //ver a que equipa pertenço
            if(red_team->playerBelongsToTeam(name))
            {
                my_team = red_team;
                my_preys = green_team;
                my_hunters = blue_team;
                setTeamName("red");

            }
            else if(green_team->playerBelongsToTeam(name))
            {
                my_team = green_team;
                my_preys = blue_team;
                my_hunters = red_team;
                setTeamName("green");

            }
            else if(blue_team->playerBelongsToTeam(name))
            {
                my_team = blue_team;
                my_preys = red_team;
                my_hunters = green_team;
                setTeamName("blue");

            }

            sub = boost::shared_ptr<ros::Subscriber> (new ros::Subscriber());
            *sub = n.subscribe("/make_a_play", 100, &MyPlayer::move, this);
            
            // move to a random place for the first time
            warp(randomizePosition(), randomizePosition(),M_PI/2);

            printReport();
        }


        void warp(double x, double y, double alfa)
        {
            tf::Transform transform;    //declare the transformation object
            transform.setOrigin( tf::Vector3(x,y, 0.0) );
            tf::Quaternion q;   
            q.setRPY(0, 0, alfa); 
            transform.setRotation(q);   
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "jmota"));
            ROS_INFO("Warping to x=%f y=%f a=%f", x, y, alfa);  
        }

        void move(const rws2018_msgs::MakeAPlay::ConstPtr& msg)
        {
            tf::Transform transform;    //declare the transformation object
            // rx = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
            // ry = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

            double x = transform.getOrigin().x();
            double y = transform.getOrigin().y();
            double a = 0;


            // if (x>=-5.0 && x<=0.0){ x+=0.01;}
            // else if (x<-5.0) {x=x+3.0;}
            // else {x+=0.01;}
            // if (y<=5.0 && y>=0.0){ y+=0.01;}
            // else if (y>5.0) {y=y-3.0;}
            // else {y+=0.01;}
            
            if (x<=5.0)
            {
                transform.setOrigin( tf::Vector3(x+=0.01,y+=0.01, 0.0) );
            }
            


            tf::Quaternion q;   
            q.setRPY(0, 0, M_PI/3); 
            transform.setRotation(q);   
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "jmota"));  
        }

        void printReport()
        {
            //cout << "My name is " << name <<  " and my team is " << getTeamName() << endl;
            ROS_INFO("My name is %s and my team is %s and I am trying to move.",name.c_str(), getTeamName().c_str());
            
        }
    };


};  //end of namespace rws_jmota

int main(int argc, char **argv)
{
    ros::init(argc, argv, "jmota");

    // string player_name = "jmota";
    // //Creating an instance of class Player
    // Player player(player_name);

    // player.setTeamName("red");
    // player.setTeamName(1);
                                                                                                        
    // cout << "player.name is " << player.name << endl;
    // cout << "team is " << player.getTeam() << endl;

    //Creating an instance of class Player
    rws_jmota::MyPlayer my_player("jmota","blue");
    // cout << "my_player.name is " << my_player.name << endl;
    // cout << "team is " << my_player.getTeamName() << endl; 

    //Creating an instance of class Team
    // rws_jmota::Team team("pink");


    // team.players.push_back("jmota");
    // team.players.push_back("vsantos");
    // //team.players[2]="lsarmento";
      
    // team.printTeamInfo();

    // ros::NodeHandle node;
    // string test_param_value;
    // node.getParam("test_param", test_param_value);

    // cout << "read test_param with value " << test_param_value << endl;


    // if (my_player.red_team->playerBelongsToTeam("amartins"))
    // {
    //     cout << "a joana esta na equipa certa" << endl;
    // };

    ros::NodeHandle n;

   // ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);
    // ros::Rate loop_rate(10); //Number of messages per second
    // while (ros::ok())
    // {
    //     my_player.move();
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    ros::spin();
}