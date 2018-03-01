//System includes
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

//Boost includes
#include <boost/shared_ptr.hpp>
#include <rws2018_libs/team.h>

  
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
                    cout << "wrong team index given. Cannot set team" << endl; break;
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
                cout << "cannot set team name to " << argin_team << endl;
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

    // /**
    //  * @brief Class myPlayer extends class Player
    //  */
    // /**
    //  * @brief Contains a description of a team
    //  */
    // class Team 
    // {
    //     public: 
        
    //     /**
    //      * @brief Constructor
    //      * @param team_name the team name
    //      */
    //     Team(string team_name)
    //     {
    //         name = team_name; 
    //     }

    //     /**
    //      * @brief Prints the name of the team and the names of all its players
    //      */
    //     void printTeamInfo(void)
    //     {
    //         cout << "team is " << name << endl; 
    //         cout << "team players are " << players[0] << " and " << players[1] << endl;  
    //         //Write code here ...
    //     }

    //     /**
    //      * @brief Checks if a player belongs to the team
    //      * @param player_name the name of the player to check
    //      * @return true or false, yes or no
    //      */
    //     bool playerBelongsToTeam(string player_name)
    //     {
    //         //write code here ...
    //     }

    //     /**
    //      * @brief The team name
    //      */
    //     string name;

    //     /**
    //      * @brief A list of the team's player names
    //      */
    //     vector<string> players;

    // };//end of class Team

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

            printReport();
        }

        void move(void)
        {
            tf::Transform transform;    //declare the transformation object 
            transform.setOrigin( tf::Vector3(5,5, 0.0) );   
            tf::Quaternion q;   
            q.setRPY(0, 0, M_PI/3); 
            transform.setRotation(q);   
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "jmota"));  
        }

        void printReport()
        {
            cout << "My name is " << name <<  " and my team is " << getTeamName() << endl;
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


    if (my_player.red_team->playerBelongsToTeam("amartins"))
    {
        cout << "a joana esta na equipa certa" << endl;
    };

    ros::NodeHandle n;

   // ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);
    ros::Rate loop_rate(10); //Number of messages per second
    while (ros::ok())
    {
        my_player.move();
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();
}