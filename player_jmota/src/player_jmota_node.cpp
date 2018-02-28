#include <iostream>                                                                                     

class Player
{
    public:

    //Constructor with the same name as the class
    Player(std::string name)
    {
        this->name = name;
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

int main()
{

    std::string player_name = "jmota";
    //Creating an instance of class Player
    Player player(player_name);
    player.setTeamName("red");
                                                                                                        
    std::cout << "player.name is " << player.name << std::endl;
    std::cout << "team is " << player.getTeam() << std::endl;
}