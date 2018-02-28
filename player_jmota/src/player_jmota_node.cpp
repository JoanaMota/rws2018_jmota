//System includes
#include <iostream>
#include <vector>

//Boost includes
#include <boost/shared_ptr.hpp>
  
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
        Player(std::string name)
        {
            this->name = name;
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
                    std::cout << "wrong team index given. Cannot set team" << std::endl; break;
            }
        }

        //Set team name, if given a correct team name (accessor)
        int setTeamName(std::string team)
        {
            if (team=="red" || team=="green" || team=="blue")
            {
                this->team = team;
                return 1;
            }
            else
            {
                std::cout << "cannot set team name to " << team << std::endl;
                return 0;
            }
        }

        //Gets team name (accessor)
        std::string getTeam(void) {return team;}

        //A public atribute
        std::string name;

        private:
        std::string team;

    };

    /**
     * @brief Class myPlayer extends class Player
     */
    class myPlayer: public Player
    {
        public: 
                                                                                                            
        myPlayer(std::string name, std::string team): Player(name)
        {
            setTeamName(team);
        }
    };

    /**
     * @brief Contains a description of a team
     */
    class Team
    {
        public: 
        
        /**
         * @brief Constructor
         * @param team_name the team name
         */
        Team(string team_name)
        {
            name = team_name; 

        }

        /**
         * @brief Prints the name of the team and the names of all its players
         */
        void printTeamInfo(void)
        {
            cout << "team is " << name << endl; 
            cout << "team players are " << name << endl;  
            //Write code here ...
        }

        /**
         * @brief Checks if a player belongs to the team
         * @param player_name the name of the player to check
         * @return true or false, yes or no
         */
        bool playerBelongsToTeam(string player_name)
        {
            //write code here ...
        }

        /**
         * @brief The team name
         */
        string name;

        /**
         * @brief A list of the team's player names
         */
        vector<string> players;

    };//end of class Team

};  //end of namespace rws_jmota

int main()
{

    // std::string player_name = "jmota";
    // //Creating an instance of class Player
    // Player player(player_name);

    // player.setTeamName("red");
    // player.setTeamName(1);
                                                                                                        
    // std::cout << "player.name is " << player.name << std::endl;
    // std::cout << "team is " << player.getTeam() << std::endl;

    //Creating an instance of class Player
    rws_jmota::myPlayer my_player("jmota","green");
    cout << "my_player.name is " << my_player.name << endl;
    cout << "team is " << my_player.getTeam() << endl; 

    //Creating an instance of class Team
    rws_jmota::Team team("blue");

    team.players.push_back("moliveira");
    team.players.push_back("vsantos");
      
    team.printTeamInfo();
}